#ifndef __IMU_GNSS_ESKF_H__
#define __IMU_GNSS_ESKF_H__

#include "eigen_type/eigen_types.h"
#include "math_utils.h"
#include "navigation_and_mapping/gnss.h"
#include "navigation_and_mapping/imu.h"
#include "navigation_and_mapping/nav_state.h"
#include "navigation_and_mapping/odom.h"

namespace sad {

/**
 * ESKF误差状态卡尔曼滤波器
 * 可以指定观测GNSS的读数，GNSS应该事先转换到车体坐标系
 *
 * 本书使用18维的ESKF，标量类型可以由S指定，默认取double
 * 变量顺序：p, v, R, bg, ba, grav，名义状态量
 * @tparam S    状态变量的精度，取float或double
 */

template <typename S = double> class ESKF {
public:
  /// 类型定义
  using SO3 = Sophus::SO3;                       // 旋转变量类型
  using VecT = Eigen::Matrix<S, 3, 1>;           // 向量类型
  using Vec18T = Eigen::Matrix<S, 18, 1>;        // 18维向量类型
  using Mat3T = Eigen::Matrix<S, 3, 3>;          // 3x3矩阵类型
  using MotionNoiseT = Eigen::Matrix<S, 18, 18>; // 运动噪声类型
  using OdomNoiseT = Eigen::Matrix<S, 3, 3>;     // 里程计噪声类型
  using GnssNoiseT = Eigen::Matrix<S, 6, 6>;     // GNSS噪声类型
  using Mat18T = Eigen::Matrix<S, 18, 18>; // 18维方差类型 p, v, R, bg, ba, grav
                                           // 每个状态量有3维，3*6
  using NavStateT = NavState<S>; // 整体名义状态变量类型

  struct Options {
    Options() = default;

    /// IMU 测量与零偏参数
    double imu_dt = 0.01; // IMU测量间隔
    // NOTE IMU噪声项都为离散时间，不需要再乘dt，可以由初始化器指定IMU噪声
    double gyro_var = 1e-5;      // 陀螺测量标准差
    double acce_var = 1e-2;      // 加计测量标准差
    double bias_gyro_var = 1e-6; // 陀螺零偏游走标准差
    double bias_acce_var = 1e-4; // 加计零偏游走标准差

    /// 里程计参数
    double odom_var = 0.5;
    double odom_span = 0.1;       // 里程计测量间隔
    double wheel_radius = 0.155;  // 轮子半径
    double circle_pulse = 1024.0; // 编码器每圈脉冲数

    /// RTK 观测参数
    double gnss_pos_noise = 0.1;                  // GNSS位置噪声
    double gnss_height_noise = 0.1;               // GNSS高度噪声
    double gnss_ang_noise = 1.0 * math::kDEG2RAD; // GNSS旋转噪声

    /// 其他配置
    bool update_bias_gyro = true; // 是否更新陀螺bias
    bool update_bias_acce = true; // 是否更新加计bias
  };

  /// @brief 初始零偏取零
  /// @param option
  ESKF(Options option = Options()) : options_(option) { BuildNoise(option); }

  /**
   * 初始化各类信息矩阵
   * @param options 噪声项配置
   * @param init_bg 初始零偏 陀螺
   * @param init_ba 初始零偏 加计
   * @param gravity 重力
   */
  void SetInitialConditions(Options options, const VecT &init_bg,
                            const VecT &init_ba,
                            const VecT &gravity = VecT(0, 0, -9.8)) {
    BuildNoise(options);
    options_ = options;
    bg_ = init_bg;
    ba_ = init_ba;
    g_ = gravity;
    cov_ = Mat18T::Identity() * 1e-4;
  }

  /// 使用IMU递推
  /// 需要计算名义状态变量的更新过程以及协方差矩阵的递推
  bool Predict(const IMU &imu);

  /// 使用轮速计观测
  bool ObserveWheelSpeed(const Odom &odom);

  /// 使用GPS观测
  /// GNSS对R的观测可以直接写成对误差状态theta的观测
  bool ObserveGps(const GNSS &gnss);

  /**
   * 使用SE3进行观测
   * @param pose  观测位姿
   * @param trans_noise 平移噪声
   * @param ang_noise   角度噪声
   * @return
   */
  bool ObserveSE3(const SE3 &pose, double trans_noise = 0.1,
                  double ang_noise = 1.0 * math::kDEG2RAD);

  /// accessors
  /// 获取全量状态
  NavStateT GetNominalState() const {
    return NavStateT(current_time_, R_, p_, v_, bg_, ba_);
  }

  /// 获取SE3 状态
  SE3 GetNominalSE3() const { return SE3(R_, p_); }

  /// 设置状态X
  void SetX(const NavStated &x, const Vec3d &grav) {
    current_time_ = x.timestamp_;
    R_ = x.R_;
    p_ = x.p_;
    v_ = x.v_;
    bg_ = x.bg_;
    ba_ = x.ba_;
    g_ = grav;
  }

  /// 设置协方差
  void SetCov(const Mat18T &cov) { cov_ = cov; }

  /// 获取重力
  Vec3d GetGravity() const { return g_; }

private:
  void BuildNoise(const Options &options) {
    double ev = options.acce_var;
    double et = options.gyro_var;
    double eg = options.bias_gyro_var;
    double ea = options.bias_acce_var;

    double ev2 = ev; // * ev;
    double et2 = et; // * et;
    double eg2 = eg; // * eg;
    double ea2 = ea; // * ea;

    // 设置imu初始噪声协方差矩阵
    // 设置过程噪声
    Q_.diagonal() << 0, 0, 0, ev2, ev2, ev2, et2, et2, et2, eg2, eg2, eg2, ea2,
        ea2, ea2, 0, 0, 0;

    // 设置里程计噪声
    double o2 = options_.odom_var * options_.odom_var;
    odom_noise_.diagonal() << o2, o2, o2;

    // 设置GNSS状态
    double gp2 = options.gnss_pos_noise * options.gnss_pos_noise;
    double gh2 = options.gnss_height_noise * options.gnss_height_noise;
    double ga2 = options.gnss_ang_noise * options.gnss_ang_noise;
    gnss_noise_.diagonal() << gp2, gp2, gh2, ga2, ga2, ga2;
  }

  /// @brief 更新名义状态变量，重置error state
  void UpdateAndReset() {
    p_ += dx_.template block<3, 1>(0, 0);
    v_ += dx_.template block<3, 1>(3, 0);
    R_ = R_ * SO3::exp(dx_.template block<3, 1>(6, 0));

    if (options_.update_bias_gyro) {
      bg_ += dx_.template block<3, 1>(9, 0);
    }

    if (options_.update_bias_acce) {
      ba_ += dx_.template block<3, 1>(12, 0);
    }

    g_ += dx_.template block<3, 1>(15, 0);

    // 重置ESKF的均值和协方差矩阵
    ProjectCov();
    dx_.setZero();
  }

  /// @brief 对P阵进行投影，把一个切空间中的高斯分布投影到另一个切空间
  void ProjectCov() {
    Mat18T J = Mat18T::Identity();
    J.template block<3, 3>(6, 6) =
        Mat3T::Identity() - 0.5 * SO3::hat(dx_.template block<3, 1>(6, 0));
    cov_ = J * cov_ * J.transpose();
  }

  /// 成员变量
  double current_time_ = 0.0; // 当前时间

  /// 名义状态
  VecT p_ = VecT::Zero();
  VecT v_ = VecT::Zero();
  SO3 R_;
  VecT bg_ = VecT::Zero();
  VecT ba_ = VecT::Zero();
  VecT g_{0, 0, -9.8};

  /// 误差状态
  Vec18T dx_ = Vec18T::Zero();

  /// 协方差阵
  Mat18T cov_ = Mat18T::Identity();

  /// 噪声阵
  MotionNoiseT Q_ = MotionNoiseT::Zero();
  OdomNoiseT odom_noise_ = OdomNoiseT::Zero();
  GnssNoiseT gnss_noise_ = GnssNoiseT::Zero();

  /// 标志位
  bool first_gnss_ = true; // 是否为第一个gnss数据

  /// 配置项
  Options options_;
};

using ESKFD = ESKF<double>;
using ESKFF = ESKF<float>;

template <typename S> bool ESKF<S>::Predict(const IMU &imu) {
  assert(imu.timestamp_ >= current_time_);

  double dt = imu.timestamp_ - current_time_;
  if (dt > (5 * options_.imu_dt) || dt < 0) {
    // 时间间隔不对，可能是第一个IMU数据，没有历史信息
    LOG(INFO) << "skip this imu because dt_ = " << dt;
    current_time_ = imu.timestamp_;
    return false;
  }

  // nominal state 名义状态变量的更新
  // 离散时间运动方程
  VecT new_p = p_ + v_ * dt + 0.5 * (R_ * (imu.acce_ - ba_)) * dt * dt +
               0.5 * g_ * dt * dt;
  VecT new_v = v_ + R_ * (imu.acce_ - ba_) * dt + g_ * dt;
  SO3 new_R = R_ * SO3::exp((imu.gyro_ - bg_) * dt);

  R_ = new_R;
  v_ = new_v;
  p_ = new_p;
  // 其余状态维度不变

  // error state 递推
  // 计算误差状态变量运动过程
  // 计算运动过程雅可比矩阵 F，见(3.47)
  // F实际上是稀疏矩阵，也可以不用矩阵形式进行相乘而是写成散装形式，这里为了教学方便，使用矩阵形式
  Mat18T F = Mat18T::Identity();                         // 主对角线
  F.template block<3, 3>(0, 3) = Mat3T::Identity() * dt; // p 对 v
  F.template block<3, 3>(3, 6) = -R_.matrix() * SO3::hat(imu.acce_ - ba_) *
                                 dt; // v对theta 李代数so3 -> 三维反对称矩阵
  F.template block<3, 3>(3, 12) = -R_.matrix() * dt;      // v 对 ba
  F.template block<3, 3>(3, 15) = Mat3T::Identity() * dt; // v 对 g
  F.template block<3, 3>(6, 6) =
      SO3::exp(-(imu.gyro_ - bg_) * dt).matrix(); // theta 对 theta 李代数转李群
  F.template block<3, 3>(6, 9) = -Mat3T::Identity() * dt; // theta 对 bg

  // ESKF update
  // mean and cov prediction
  // 下行其实没必要算，dx_在重置之后应该为零，因此这步可以跳过，但F需要参与Cov部分计算，所以保留
  dx_ = F * dx_;
  // 在Eigen中，当变量同时出现在左值和右值，赋值操作可能会带来混淆问题。
  // Eigen需要把右值赋值为一个临时matrix/array，然后再将临时值赋值给左值，便可以解决混淆。eval()函数实现了这个功能，可以解决混淆问题。
  cov_ = F * cov_.eval() * F.transpose() + Q_;
  current_time_ = imu.timestamp_;
  return true;

  // 预测的实质是使用imu的读数对名义状态进行递推,同时在协方差矩阵中合入运动过程噪声，这样协方差矩阵回慢慢变大
  // 而误差状态会在观测过程置零 F * dx_ 为0
}

template <typename S> bool ESKF<S>::ObserveWheelSpeed(const Odom &odom) {
  assert(odom.timestamp_ >= current_time_);
  // odom 修正以及雅可比
  // 使用三维的轮速观测，H为3x18，大部分为零
  Eigen::Matrix<S, 3, 18> H = Eigen::Matrix<S, 3, 18>::Zero();
  H.template block<3, 3>(0, 3) = Mat3T::Identity();

  // 卡尔曼增益
  Eigen::Matrix<S, 18, 3> K =
      cov_ * H.transpose() * (H * cov_ * H.transpose() + odom_noise_).inverse();

  // velocity obs
  double velo_l = options_.wheel_radius * odom.left_pulse_ /
                  options_.circle_pulse * 2 * M_PI / options_.odom_span;
  double velo_r = options_.wheel_radius * odom.right_pulse_ /
                  options_.circle_pulse * 2 * M_PI / options_.odom_span;
  double average_vel = 0.5 * (velo_l + velo_r);

  VecT vel_odom(average_vel, 0.0, 0.0);
  VecT vel_world = R_ * vel_odom;

  dx_ = K * (vel_world - v_);

  // update cov
  cov_ = (Mat18T::Identity() - K * H) * cov_;

  UpdateAndReset();
  return true;
}

template <typename S> bool ESKF<S>::ObserveGps(const GNSS &gnss) {
  /// GNSS 观测的修正
  assert(gnss.unix_time_ >= current_time_);

  if (first_gnss_) {
    R_ = gnss.utm_pose_.so3();
    p_ = gnss.utm_pose_.translation();
    first_gnss_ = false;
    current_time_ = gnss.unix_time_;
    return true;
  }

  assert(gnss.heading_valid_);
  // 使用SE3进行观测
  ObserveSE3(gnss.utm_pose_, options_.gnss_pos_noise, options_.gnss_ang_noise);
  current_time_ = gnss.unix_time_;

  return true;
}

template <typename S>
bool ESKF<S>::ObserveSE3(const SE3 &pose, double trans_noise,
                         double ang_noise) {
  // 这里认为RTK可以提供6自由度的观测（双天线）
  /// 既有旋转，也有平移
  /// 观测状态变量中的p, R，H为6x18，其余为零
  Eigen::Matrix<S, 6, 18> H = Eigen::Matrix<S, 6, 18>::Zero();
  H.template block<3, 3>(0, 0) = Mat3T::Identity(); // P部分
  H.template block<3, 3>(3, 6) = Mat3T::Identity(); // R部分（3.66)

  // 卡尔曼增益和更新过程
  Vec6d noise_vec;
  noise_vec << trans_noise, trans_noise, trans_noise, ang_noise, ang_noise,
      ang_noise;

  // gnss的噪声协方差矩阵
  Mat6d V = noise_vec.asDiagonal();

  // 计算kalman gain
  Eigen::Matrix<S, 18, 6> K =
      cov_ * H.transpose() * (H * cov_ * H.transpose() + V).inverse();

  // 更新x和cov
  Vec6d innov = Vec6d::Zero();
  innov.template head<3>() = (pose.translation() - p_); // 平移部分
  innov.template tail<3>() =
      (R_.inverse() * pose.so3()).log(); // 旋转部分(3.67)

  dx_ = K * innov;
  cov_ = (Mat18T::Identity() - K * H) * cov_;

  // 后续处理
  // 将误差状态归入名义状态
  UpdateAndReset();

  // RTK读数主要是在观测阶段通过卡尔曼增益作用于误差状态变量中
  return true;
}

} // namespace sad

#endif // __IMU_GNSS_ESKF_H__
