#ifndef BASED_IMU_INTEGRATION_GINS_H
#define BASED_IMU_INTEGRATION_GINS_H

#include <deque>
#include <fstream>
#include <memory>

#include "eigen_type/eigen_types.h"
#include "math_utils.h"
#include "navigation_and_mapping/gnss.h"
#include "navigation_and_mapping/imu.h"
#include "navigation_and_mapping/odom.h"

#include "imu_preintegration.h"

namespace sad {
class GinsImuPreIntegration {
public:
  /// GINS 配置项
  struct Options {
    Options() {}

    Vec3d gravity = Vec3d(0, 0, -9.8); // 重力方向

    /// IMU相关噪声参数在preinteg内部配置
    IMUPreintegration::Options preinteg_options;

    // 噪声
    double bias_gyro_var = 1e-6;          // 陀螺零偏游走标准差
    double bias_acce_var = 1e-4;          // 加计零偏游走标准差
    Mat3d bg_rw_info = Mat3d::Identity(); // 陀螺随机游走信息阵
    Mat3d ba_rw_info = Mat3d::Identity(); // 加计随机游走信息阵

    double gnss_pos_noise = 0.1;                  // GNSS位置方差
    double gnss_height_noise = 0.1;               // GNSS高度方差
    double gnss_ang_noise = 1.0 * math::kDEG2RAD; // GNSS角度方差
    Mat6d gnss_info = Mat6d::Identity();          // 6D gnss信息矩阵

    /// 轮速计相关
    double odom_var = 0.05;
    Mat3d odom_info = Mat3d::Identity();
    double odom_span = 0.1;       // 里程计测量间隔
    double wheel_radius = 0.155;  // 轮子半径
    double circle_pulse = 1024.0; // 编码器每圈脉冲数

    bool verbose_ = true; // 是否输出调试信息
  };

  GinsImuPreIntegration(Options options = Options());
  ~GinsImuPreIntegration();

  /// @brief add imu data
  /// @note 需要设置初始偏移后在调用
  /// @param imu
  void AddImu(const IMU &imu);

  /// @brief add gnss data
  /// @param gnss
  void AddGnss(const GNSS &gnss);

  /// @brief add odom data
  /// @param odom
  void AddOdom(const Odom &odom);

  /// 设置gins的各种配置项，可以在构建时调用，也可以构造完成后，静止初始化结束时调用
  void SetOptions(Options options);

  /// @brief get current state, 如果IMU没有积分，就返回最后优化的状态,
  /// 如果有，利用IMU积分，预测自己的状态
  /// @return
  NavStated GetState() const;

private:
  // g2o优化
  void Optimize();

  Options options_;
  double current_time_ = 0.0; // 当前时间

  std::shared_ptr<IMUPreintegration> pre_integ_ = nullptr;
  std::shared_ptr<NavStated> last_frame_ = nullptr; // 上一个时刻状态
  std::shared_ptr<NavStated> this_frame_ = nullptr; // 当前时刻状态
  Mat15d prior_info_ = Mat15d::Identity() * 1e2;    // 当前时刻先验

  /// 两帧GNSS观测
  GNSS last_gnss_;
  GNSS this_gnss_;

  IMU last_imu_;   // 上时刻IMU
  Odom last_odom_; // 上时刻odom
  bool last_odom_set_ = false;

  /// 标志位
  bool first_gnss_received_ = false; // 是否已收到第一个gnss信号
  bool first_imu_received_ = false;  // 是否已收到第一个imu信号
};
} // namespace sad

#endif
