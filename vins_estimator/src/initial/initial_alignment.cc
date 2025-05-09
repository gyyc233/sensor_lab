#include "initial_alignment.h"

namespace sensor_lab {

/// @brief 通过视觉与 IMU 数据联合估计陀螺仪偏置（gyroscope
/// bias）的初始值，并更新所有帧的预积分数据
/// @note 利用视觉估计的帧间旋转（q_ij）与 IMU
/// 预积分中的预测旋转之间的差异，通过最小二乘，反推出最可能的陀螺仪偏置
/// @param all_image_frame 图像帧集合
/// @param Bgs 每帧对应的陀螺仪偏置
void solveGyroscopeBias(std::map<double, ImageFrame> &all_image_frame,
                        Eigen::Vector3d *Bgs) {
  // 在 VIO 中，IMU
  // 的陀螺仪测量中通常包含一个缓慢变化的偏置（bias）。这个偏置如果不准确，会导致预积分误差累积，从而影响位姿估计
  Eigen::Matrix3d A;
  Eigen::Vector3d b;
  Eigen::Vector3d delta_bg; // 方程增量
  A.setZero();
  b.setZero();
  std::map<double, ImageFrame>::iterator frame_i;
  std::map<double, ImageFrame>::iterator frame_j;
  for (frame_i = all_image_frame.begin();
       next(frame_i) != all_image_frame.end(); frame_i++) {
    // 每次处理相邻两帧
    frame_j = next(frame_i);
    Eigen::MatrixXd tmp_A(3, 3);
    tmp_A.setZero();
    Eigen::VectorXd tmp_b(3);
    tmp_b.setZero();

    // 使用视觉估计，计算frame_j ->frame_i的旋转矩阵
    Eigen::Quaterniond q_ij(frame_i->second.R.transpose() * frame_j->second.R);
    // imu与积分种旋转部分对陀螺仪偏置的雅可比矩阵
    tmp_A = frame_j->second.pre_integration->jacobian.template block<3, 3>(
        O_R, O_BG);
    // 计算 IMU
    // 预测的旋转与视觉估计的旋转之间的误差（四元数），提取虚部作为误差向量
    tmp_b =
        2 * (frame_j->second.pre_integration->delta_q.inverse() * q_ij).vec();
    // 构建gaussian-newton增量方程
    A += tmp_A.transpose() * tmp_A; // J^T * J 近似H矩阵
    b += tmp_A.transpose() * tmp_b; // J^T * r 近似b向量
  }
  delta_bg = A.ldlt().solve(b); //  LDLT 分解求解线性系统的最小二乘解
  std::cout << "gyroscope bias initial calibration " << delta_bg.transpose()
            << std::endl;

  // 更新每帧的陀螺仪偏置
  for (int i = 0; i <= WINDOW_SIZE; i++)
    Bgs[i] += delta_bg;

  // 重新传播预积分数据
  for (frame_i = all_image_frame.begin();
       next(frame_i) != all_image_frame.end(); frame_i++) {
    frame_j = next(frame_i);
    // 使用新的偏置重新计算每一帧的 IMU 预积分
    frame_j->second.pre_integration->repropagate(Vector3d::Zero(), Bgs[0]);
  }
}

/// @brief 构造一个与重力向量g_0正交的二维切空间基底（tangent
/// basis），即返回两个单位正交向量，它们构成重力方向的切平面上的一组基
/// @param g0 当前估计的重力向量
/// @return
Eigen::MatrixXd TangentBasis(Eigen::Vector3d &g0) {
  // 将重力方向限制在单位球面上，在当前点处构造切空间，扰动在这个切空间中进行
  // 这样可以保证每次更新后仍然是单位向量方向，并且扰动具有明确几何含义

  // 希望找到两个单位向量 b 和 c，他们都与归一化后的 g0 垂直，且b c
  // 相互正交，构成右手系
  Eigen::Vector3d b, c;
  Eigen::Vector3d a = g0.normalized();

  Eigen::Vector3d tmp(0, 0, 1);
  // 如果 a 恰好是 (0, 0, 1)，则换用 (1, 0, 0) 避免退化
  if (a == tmp)
    tmp << 1, 0, 0;

  // Gram-Schmidt 正交化
  b = (tmp - a * (a.transpose() * tmp)).normalized();
  // 构造第二个切向量 c
  c = a.cross(b);
  Eigen::MatrixXd bc(3, 2);
  bc.block<3, 1>(0, 0) = b;
  bc.block<3, 1>(0, 1) = c;
  return bc;
}

/// @brief 在固定尺度的前提下，通过非线性优化（高斯牛顿）逐步
/// refine（精细化）重力方向的估计值
/// @param all_image_frame
/// @param g 当前重力向量
/// @param x 状态变量
void RefineGravity(std::map<double, ImageFrame> &all_image_frame,
                   Eigen::Vector3d &g, Eigen::VectorXd &x) {
  // 重力向量归一化并乘以标准重力模长
  Eigen::Vector3d g0 = g.normalized() * G.norm();
  Eigen::Vector3d lx, ly;
  // VectorXd x;
  int all_frame_count = all_image_frame.size();
  int n_state = all_frame_count * 3 + 2 + 1;

  Eigen::MatrixXd A{n_state, n_state};
  A.setZero();
  Eigen::VectorXd b{n_state};
  b.setZero();

  std::map<double, ImageFrame>::iterator frame_i;
  std::map<double, ImageFrame>::iterator frame_j;

  // 迭代四次
  // 若数据质量较差（如 IMU
  // 噪声大、视觉跟踪不稳定），重力方向误差大，初始估计不好，则需要增加迭代次数
  for (int k = 0; k < 4; k++) {
    Eigen::MatrixXd lxly(3, 2); // 重力向量的切空间基底
    lxly = TangentBasis(g0);
    int i = 0;

    for (frame_i = all_image_frame.begin();
         next(frame_i) != all_image_frame.end(); frame_i++, i++) {
      // 对每个连续帧对构建残差
      frame_j = next(frame_i);

      Eigen::MatrixXd tmp_A(6, 9); // jacobian
      tmp_A.setZero();
      Eigen::VectorXd tmp_b(6); // residual
      tmp_b.setZero();

      double dt = frame_j->second.pre_integration->sum_dt;

      // 关于速度的偏导
      tmp_A.block<3, 3>(0, 0) = -dt * Eigen::Matrix3d::Identity();
      // 关于重力扰动方向的偏导
      tmp_A.block<3, 2>(0, 6) = frame_i->second.R.transpose() * dt * dt / 2 *
                                Eigen::Matrix3d::Identity() * lxly;

      // 关于尺度的偏导
      tmp_A.block<3, 1>(0, 8) = frame_i->second.R.transpose() *
                                (frame_j->second.T - frame_i->second.T) / 100.0;

      // IMU 预测与视觉观测之间的位移差异
      tmp_b.block<3, 1>(0, 0) =
          frame_j->second.pre_integration->delta_p +
          frame_i->second.R.transpose() * frame_j->second.R * TIC[0] - TIC[0] -
          frame_i->second.R.transpose() * dt * dt / 2 * g0;

      // 关于前一帧速度的偏导
      tmp_A.block<3, 3>(3, 0) = -Eigen::Matrix3d::Identity();

      // 关于后一帧旋转的偏导
      tmp_A.block<3, 3>(3, 3) =
          frame_i->second.R.transpose() * frame_j->second.R;

      // 关于重力扰动的偏导
      tmp_A.block<3, 2>(3, 6) = frame_i->second.R.transpose() * dt *
                                Eigen::Matrix3d::Identity() * lxly;

      // IMU 预测与视觉观测之间的速度差异
      tmp_b.block<3, 1>(3, 0) =
          frame_j->second.pre_integration->delta_v -
          frame_i->second.R.transpose() * dt * Eigen::Matrix3d::Identity() * g0;

      Eigen::Matrix<double, 6, 6> cov_inv = Eigen::Matrix<double, 6, 6>::Zero();
      // cov.block<6, 6>(0, 0) = IMU_cov[i + 1];
      // MatrixXd cov_inv = cov.inverse();
      cov_inv.setIdentity();

      // 计算增量方程,协方差矩阵 cov_inv 对残差进行加权
      Eigen::MatrixXd r_A = tmp_A.transpose() * cov_inv * tmp_A;
      Eigen::VectorXd r_b = tmp_A.transpose() * cov_inv * tmp_b;

      A.block<6, 6>(i * 3, i * 3) += r_A.topLeftCorner<6, 6>();
      b.segment<6>(i * 3) += r_b.head<6>();

      A.bottomRightCorner<3, 3>() += r_A.bottomRightCorner<3, 3>();
      b.tail<3>() += r_b.tail<3>();

      A.block<6, 3>(i * 3, n_state - 3) += r_A.topRightCorner<6, 3>();
      A.block<3, 6>(n_state - 3, i * 3) += r_A.bottomLeftCorner<3, 6>();
    }
    A = A * 1000.0;
    b = b * 1000.0;
    x = A.ldlt().solve(b);
    // dg: 是切空间中的扰动方向（2D），表示重力方向应如何调整
    Eigen::VectorXd dg = x.segment<2>(n_state - 3);
    std::cout << "dg: " << dg.transpose() << std::endl;

    if (dg.norm() < 1e-6)
      break;
    // lxly * dg: 把切空间扰动映射到三维空间
    // g0: 更新后的重力方向（保持模长不变）
    g0 = (g0 + lxly * dg).normalized() * G.norm();
    // double s = x(n_state - 1);
  }
  g = g0;
}

/// @brief visual-IMU 线性对齐
/// @note gaussian-newton
/// 构建残差项,使用LDLT求解Ax=b，优化状态变量（包括重力向量与尺度因子）
/// @param all_image_frame 图像帧集合
/// @param g output 重力向量
/// @param x output 优化后的状态向
/// @return
bool LinearAlignment(std::map<double, ImageFrame> &all_image_frame,
                     Eigen::Vector3d &g, Eigen::VectorXd &x) {
  int all_frame_count = all_image_frame.size();
  // 定义状态变量维度：每帧的速度增量*3 + 3（重力方向）+ 1（尺度）
  int n_state = all_frame_count * 3 + 3 + 1;

  Eigen::MatrixXd A{n_state, n_state};
  A.setZero();
  Eigen::VectorXd b{n_state};
  b.setZero();

  std::map<double, ImageFrame>::iterator frame_i;
  std::map<double, ImageFrame>::iterator frame_j;
  int i = 0;

  for (frame_i = all_image_frame.begin();
       next(frame_i) != all_image_frame.end(); frame_i++, i++) {
    frame_j = next(frame_i);

    Eigen::MatrixXd tmp_A(6, 10);
    tmp_A.setZero();
    Eigen::VectorXd tmp_b(6);
    tmp_b.setZero();

    double dt = frame_j->second.pre_integration->sum_dt;

    // 1. 位置残差
    // 关于前一帧速度的偏导
    tmp_A.block<3, 3>(0, 0) = -dt * Matrix3d::Identity();
    // 关于重力方向的偏导
    tmp_A.block<3, 3>(0, 6) =
        frame_i->second.R.transpose() * dt * dt / 2 * Matrix3d::Identity();
    // 关于尺度的偏导
    tmp_A.block<3, 1>(0, 9) = frame_i->second.R.transpose() *
                              (frame_j->second.T - frame_i->second.T) / 100.0;

    // IMU 预测位移与视觉观测之间的差异
    tmp_b.block<3, 1>(0, 0) =
        frame_j->second.pre_integration->delta_p +
        frame_i->second.R.transpose() * frame_j->second.R * TIC[0] - TIC[0];
    // cout << "delta_p   " <<
    // frame_j->second.pre_integration->delta_p.transpose() << endl;

    // 2. 速度残差
    // 关于前一帧速度的偏导
    tmp_A.block<3, 3>(3, 0) = -Matrix3d::Identity();
    // 关于后一帧旋转的偏导
    tmp_A.block<3, 3>(3, 3) = frame_i->second.R.transpose() * frame_j->second.R;
    // 关于重力方向的偏导
    tmp_A.block<3, 3>(3, 6) =
        frame_i->second.R.transpose() * dt * Matrix3d::Identity();
    // IMU 预测速度与视觉观测之间的差
    tmp_b.block<3, 1>(3, 0) = frame_j->second.pre_integration->delta_v;
    // cout << "delta_v   " <<
    // frame_j->second.pre_integration->delta_v.transpose() << endl;

    // 加权并构建增量方程
    Eigen::Matrix<double, 6, 6> cov_inv = Eigen::Matrix<double, 6, 6>::Zero();
    // cov.block<6, 6>(0, 0) = IMU_cov[i + 1];
    // MatrixXd cov_inv = cov.inverse();
    cov_inv.setIdentity();

    Eigen::MatrixXd r_A = tmp_A.transpose() * cov_inv * tmp_A;
    Eigen::VectorXd r_b = tmp_A.transpose() * cov_inv * tmp_b;

    A.block<6, 6>(i * 3, i * 3) += r_A.topLeftCorner<6, 6>();
    b.segment<6>(i * 3) += r_b.head<6>();

    A.bottomRightCorner<4, 4>() += r_A.bottomRightCorner<4, 4>();
    b.tail<4>() += r_b.tail<4>();

    A.block<6, 4>(i * 3, n_state - 4) += r_A.topRightCorner<6, 4>();
    A.block<4, 6>(n_state - 4, i * 3) += r_A.bottomLeftCorner<4, 6>();
  }

  A = A * 1000.0;
  b = b * 1000.0;
  x = A.ldlt().solve(b);

  // 提取尺度因子 s 并缩放回物理单位
  double s = x(n_state - 1) / 100.0;
  std::cout << "estimated scale: " << s << std::endl;

  // 从状态向量中取出重力方向估计值
  g = x.segment<3>(n_state - 4);
  std::cout << "estimated g: " << g.transpose() << std::endl;
  if (fabs(g.norm() - G.norm()) > 1.0 || s < 0) {
    return false;
  }

  // 优化重力方向
  RefineGravity(all_image_frame, g, x);
  // 除以100是为了使尺度因子与其它状态变量（速度、重力等）处于相近数量级，提升数值稳定性，后面会还原
  s = (x.tail<1>())(0) / 100.0;
  std::cout << "refined scale: " << s << std::endl;
  (x.tail<1>())(0) = s;
  std::cout << " refine     " << g.transpose() << std::endl;

  // 检查尺度因子是否合理
  if (s < 0.0)
    return false;
  else
    return true;
}

bool VisualIMUAlignment(std::map<double, ImageFrame> &all_image_frame,
                        Eigen::Vector3d *Bgs, Eigen::Vector3d &g, VectorXd &x) {
  // 1. 陀螺仪偏置校正
  solveGyroscopeBias(all_image_frame, Bgs);

  // 2. 重力向量优化与尺度因子优化
  if (LinearAlignment(all_image_frame, g, x))
    return true;
  else
    return false;
}

} // namespace sensor_lab
