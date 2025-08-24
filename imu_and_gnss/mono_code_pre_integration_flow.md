- [vins mono 预积分代码实现](#vins-mono-预积分代码实现)
  - [通过预积分观测值计算下一帧PVQ状态](#通过预积分观测值计算下一帧pvq状态)
  - [预积分观测值对零偏变化量的jacobian计算](#预积分观测值对零偏变化量的jacobian计算)
  - [计算预积分误差的线性化递推方程，得到Jacobian和Covariance递推公式](#计算预积分误差的线性化递推方程得到jacobian和covariance递推公式)
  - [预积分观测值的计算](#预积分观测值的计算)
  - [正向运行](#正向运行)

# vins mono 预积分代码实现

这里讲解的顺序会参考ESKF中imu预积分使用的倒叙法进行讲解

文件位置`vins_estimator/src/factor/integration_base.h`

## 通过预积分观测值计算下一帧PVQ状态

当陀螺仪，加速度计零偏更新时，预积分观测量也要更新，进而假设预积分观测值对零偏变化量的一阶线性近似函数，里面包含了观测量对零偏变化量的jacobian

`evaluate`函数完成了这一功能,同时他也计算了imu预积分残差

```cpp
    /**
     * 计算和给定相邻帧状态量的残差
     * @param Pi，Qi，Vi，Bai，Bgi  [前一次预积分结果]
     * @param Pj，Qj，Vj，Baj，Bgj  [后一次预积分结果]
     * @return
     */
    Eigen::Matrix<double, 15, 1> evaluate(const Eigen::Vector3d &Pi, const Eigen::Quaterniond &Qi, const Eigen::Vector3d &Vi, const Eigen::Vector3d &Bai, const Eigen::Vector3d &Bgi,
                                          const Eigen::Vector3d &Pj, const Eigen::Quaterniond &Qj, const Eigen::Vector3d &Vj, const Eigen::Vector3d &Baj, const Eigen::Vector3d &Bgj)
    {
        Eigen::Matrix<double, 15, 1> residuals;
        // 获取P V Q 关于 ba，bg的雅克比
        Eigen::Matrix3d dp_dba = jacobian.block<3, 3>(O_P, O_BA);
        Eigen::Matrix3d dp_dbg = jacobian.block<3, 3>(O_P, O_BG);

        Eigen::Matrix3d dq_dbg = jacobian.block<3, 3>(O_R, O_BG);

        Eigen::Matrix3d dv_dba = jacobian.block<3, 3>(O_V, O_BA);
        Eigen::Matrix3d dv_dbg = jacobian.block<3, 3>(O_V, O_BG);

        Eigen::Vector3d dba = Bai - linearized_ba;
        Eigen::Vector3d dbg = Bgi - linearized_bg;

        /// 求bias改变后的预积分量
        Eigen::Quaterniond corrected_delta_q = delta_q * Utility::deltaQ(dq_dbg * dbg);
        // 对零偏雅克比线性展开
        Eigen::Vector3d corrected_delta_v = delta_v + dv_dba * dba + dv_dbg * dbg;
        Eigen::Vector3d corrected_delta_p = delta_p + dp_dba * dba + dp_dbg * dbg;

        /// 计算IMU的残差
        // 位置的残差
        residuals.block<3, 1>(O_P, 0) = Qi.inverse() * (0.5 * G * sum_dt * sum_dt + Pj - Pi - Vi * sum_dt) - corrected_delta_p;
        // 旋转的残差 ，.vec()：取出虚部
        residuals.block<3, 1>(O_R, 0) = 2 * (corrected_delta_q.inverse() * (Qi.inverse() * Qj)).vec();
        // 速度的残差
        residuals.block<3, 1>(O_V, 0) = Qi.inverse() * (G * sum_dt + Vj - Vi) - corrected_delta_v;
        // 零偏
        residuals.block<3, 1>(O_BA, 0) = Baj - Bai;
        residuals.block<3, 1>(O_BG, 0) = Bgj - Bgi;
        return residuals;
    }
```

## 预积分观测值对零偏变化量的jacobian计算

mono中预积分观测值对零偏变化量的jacobian在`midPointIntegration`函数中

- 与ESKF不同是，在计算出Jacobian之后，通过之前计算的噪声递推的系数矩阵F作为jacobian的信息矩阵，修正jacobian

```cpp
    void midPointIntegration(double _dt, 
                            const Eigen::Vector3d &_acc_0, const Eigen::Vector3d &_gyr_0,
                            const Eigen::Vector3d &_acc_1, const Eigen::Vector3d &_gyr_1,
                            const Eigen::Vector3d &delta_p, const Eigen::Quaterniond &delta_q, const Eigen::Vector3d &delta_v,
                            const Eigen::Vector3d &linearized_ba, const Eigen::Vector3d &linearized_bg,
                            Eigen::Vector3d &result_delta_p, Eigen::Quaterniond &result_delta_q, Eigen::Vector3d &result_delta_v,
                            Eigen::Vector3d &result_linearized_ba, Eigen::Vector3d &result_linearized_bg, bool update_jacobian)
    {
        // ROS_INFO("midpoint integration");
        /// step1 中值积分更新状态量(见博客2.1) 公式27
        // a = R(γ)(ai - ba) 公式（27）
        Vector3d un_acc_0 = delta_q * (_acc_0 - linearized_ba);        // 上一时刻的IMU在初始帧下的加速度的值
        // ω = 1/2(ω0 + ω1) - bw
        Vector3d un_gyr = 0.5 * (_gyr_0 + _gyr_1) - linearized_bg;     // k时刻和k+1时刻陀螺仪的值相加/2 - 零偏
        // 四元数转化为轴角：q = [cosθ/2,n * sinθ/2] ,当θ趋近于0时： θ -> 0 => q = [1,w*t/2]
        // γ_i+1 = γ_i * [1,1/2 * ω * dt ]  中值积分后的旋转
        result_delta_q = delta_q * Quaterniond(1, un_gyr(0) * _dt / 2, un_gyr(1) * _dt / 2, un_gyr(2) * _dt / 2);
        Vector3d un_acc_1 = result_delta_q * (_acc_1 - linearized_ba); // k+1 时刻的加速度值
        Vector3d un_acc = 0.5 * (un_acc_0 + un_acc_1);                 // 加速度的中值
        result_delta_p = delta_p + delta_v * _dt + 0.5 * un_acc * _dt * _dt;  // 中值积分后的位移
        result_delta_v = delta_v + un_acc * _dt;                       // 中值积分后的速度
        result_linearized_ba = linearized_ba;                           // 零偏认为不变
        result_linearized_bg = linearized_bg;         

        /// step2 更新方差矩阵和雅克比
        if(update_jacobian)
        {
            Vector3d w_x = 0.5 * (_gyr_0 + _gyr_1) - linearized_bg;
            Vector3d a_0_x = _acc_0 - linearized_ba;
            Vector3d a_1_x = _acc_1 - linearized_ba;
            Matrix3d R_w_x, R_a_0_x, R_a_1_x;
            //反对称矩阵
            R_w_x<<0, -w_x(2), w_x(1),
                w_x(2), 0, -w_x(0),
                -w_x(1), w_x(0), 0;
            R_a_0_x<<0, -a_0_x(2), a_0_x(1),
                a_0_x(2), 0, -a_0_x(0),
                -a_0_x(1), a_0_x(0), 0;
            R_a_1_x<<0, -a_1_x(2), a_1_x(1),
                a_1_x(2), 0, -a_1_x(0),
                -a_1_x(1), a_1_x(0), 0;

            // 协方差矩阵 (见博客5.0)  δx_k+1 = F * δ_k + V * n_t
            /*[ δα_k+1  ]   [ I f01 δt f03 f04 ] [ δα_k  ]   [ v00        v01      v02      v03  0  0 ][ n_ak  ]
             *[ δθ_k+1  ]   [ 0 f11  0   0 -δt ] [ δθ_k  ]   [  0       -δt/2        0    -δt/2  0  0 ][ n_ωk  ]
             *[ δβ_k+1  ] = [ 0 f21  I f23 f24 ] [ δβ_k  ] + [ -R_k*δt/2  v21  -R_k+1*δt/2  v23  0  0 ][ n_ak+1]
             *[ δb_a_k+1]   [ 0   0  0   0   0 ] [ δb_a_k]   [  0          0         0       0   0  0 ][ n_ωk+1]
             *[ δb_ω_k+1]   [ 0   0  0   0   0 ] [ δb_ω_k]   [  0          0         0       0   0  δt][ n_ba  ]
             *                                                                                         [ n_bw  ]
             */
            // F矩阵
            MatrixXd F = MatrixXd::Zero(15, 15);
            F.block<3, 3>(0, 0) = Matrix3d::Identity();
            F.block<3, 3>(0, 3) = -0.25 * delta_q.toRotationMatrix() * R_a_0_x * _dt * _dt + 
                                  -0.25 * result_delta_q.toRotationMatrix() * R_a_1_x * (Matrix3d::Identity() - R_w_x * _dt) * _dt * _dt;
            F.block<3, 3>(0, 6) = MatrixXd::Identity(3,3) * _dt;
            F.block<3, 3>(0, 9) = -0.25 * (delta_q.toRotationMatrix() + result_delta_q.toRotationMatrix()) * _dt * _dt;
            F.block<3, 3>(0, 12) = -0.25 * result_delta_q.toRotationMatrix() * R_a_1_x * _dt * _dt * -_dt;
            F.block<3, 3>(3, 3) = Matrix3d::Identity() - R_w_x * _dt;
            F.block<3, 3>(3, 12) = -1.0 * MatrixXd::Identity(3,3) * _dt;
            F.block<3, 3>(6, 3) = -0.5 * delta_q.toRotationMatrix() * R_a_0_x * _dt + 
                                  -0.5 * result_delta_q.toRotationMatrix() * R_a_1_x * (Matrix3d::Identity() - R_w_x * _dt) * _dt;
            F.block<3, 3>(6, 6) = Matrix3d::Identity();
            F.block<3, 3>(6, 9) = -0.5 * (delta_q.toRotationMatrix() + result_delta_q.toRotationMatrix()) * _dt;
            F.block<3, 3>(6, 12) = -0.5 * result_delta_q.toRotationMatrix() * R_a_1_x * _dt * -_dt;
            F.block<3, 3>(9, 9) = Matrix3d::Identity();
            F.block<3, 3>(12, 12) = Matrix3d::Identity();
            //cout<<"A"<<endl<<A<<endl;

            // 都与博客相差一个负号,因为噪声都是0均值的高斯白噪声,所以没有影响
            MatrixXd V = MatrixXd::Zero(15,18);
            V.block<3, 3>(0, 0) =  0.25 * delta_q.toRotationMatrix() * _dt * _dt;
            V.block<3, 3>(0, 3) =  0.25 * -result_delta_q.toRotationMatrix() * R_a_1_x  * _dt * _dt * 0.5 * _dt;
            V.block<3, 3>(0, 6) =  0.25 * result_delta_q.toRotationMatrix() * _dt * _dt;
            V.block<3, 3>(0, 9) =  V.block<3, 3>(0, 3);
            V.block<3, 3>(3, 3) =  0.5 * MatrixXd::Identity(3,3) * _dt;
            V.block<3, 3>(3, 9) =  0.5 * MatrixXd::Identity(3,3) * _dt;
            V.block<3, 3>(6, 0) =  0.5 * delta_q.toRotationMatrix() * _dt;
            V.block<3, 3>(6, 3) =  0.5 * -result_delta_q.toRotationMatrix() * R_a_1_x  * _dt * 0.5 * _dt;
            V.block<3, 3>(6, 6) =  0.5 * result_delta_q.toRotationMatrix() * _dt;
            V.block<3, 3>(6, 9) =  V.block<3, 3>(6, 3);
            V.block<3, 3>(9, 12) = MatrixXd::Identity(3,3) * _dt;
            V.block<3, 3>(12, 15) = MatrixXd::Identity(3,3) * _dt;

            //step_jacobian = F;
            //step_V = V;
            // 使用系数矩阵F作为jacobian矩阵的信息矩阵，更新Jacobian
            jacobian = F * jacobian;
            // 协方差的更新(博客6.3)
            covariance = F * covariance * F.transpose() + V * noise * V.transpose();
        }

    }
```

## 计算预积分误差的线性化递推方程，得到Jacobian和Covariance递推公式

这里误差的递推方程可以等价于ESKF中预积分噪声模型的递推，同样是计算两个系数矩阵`F V` 

- `F`指的是上一时刻状态量[p,v,q,b_a,b_g]的误差传递到当前时刻的系数矩阵
- `V`指的是上一时刻测量噪声传递到当前时刻的传递系数

## 预积分观测值的计算

mono中使用中值积分计算预积分观测值

```cpp
  /// step1 中值积分更新状态量(见博客2.1) 公式27
  // a = R(γ)(ai - ba) 公式（27）
  Vector3d un_acc_0 = delta_q * (_acc_0 - linearized_ba);        // 上一时刻的IMU在初始帧下的加速度的值
  // ω = 1/2(ω0 + ω1) - bw
  Vector3d un_gyr = 0.5 * (_gyr_0 + _gyr_1) - linearized_bg;     // k时刻和k+1时刻陀螺仪的值相加/2 - 零偏
  // 四元数转化为轴角：q = [cosθ/2,n * sinθ/2] ,当θ趋近于0时： θ -> 0 => q = [1,w*t/2]
  // γ_i+1 = γ_i * [1,1/2 * ω * dt ]  中值积分后的旋转
  result_delta_q = delta_q * Quaterniond(1, un_gyr(0) * _dt / 2, un_gyr(1) * _dt / 2, un_gyr(2) * _dt / 2);
  Vector3d un_acc_1 = result_delta_q * (_acc_1 - linearized_ba); // k+1 时刻的加速度值
  Vector3d un_acc = 0.5 * (un_acc_0 + un_acc_1);                 // 加速度的中值
  result_delta_p = delta_p + delta_v * _dt + 0.5 * un_acc * _dt * _dt;  // 中值积分后的位移
  result_delta_v = delta_v + un_acc * _dt;                       // 中值积分后的速度
  result_linearized_ba = linearized_ba;                           // 零偏认为不变
  result_linearized_bg = linearized_bg;
```

## 正向运行

1. 调用接口`propagate`,使用中值积分计算imu预积分观测值
2. 计算预积分误差的线性化递推方程，可用于后续Jacobian, 误差协方差的信息矩阵`midPointIntegration`
3. 计算imu预积分对零偏更新量的jacobian `midPointIntegration`, 使用信息矩阵`F V`更新Jacobian与误差协方差矩阵
4. `evaluate`基于预积分观测值与jacobian更新预积分观测值，然后计算imu`P V Q`残差
