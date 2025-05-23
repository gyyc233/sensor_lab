#include "static_imu_init.h"
#include "math_utils.h"
#include <glog/logging.h>

namespace sad {
StaticIMUInit::StaticIMUInit() {}
StaticIMUInit::StaticIMUInit(Options options = Options()) : options_(options) {}

StaticIMUInit::~StaticIMUInit() {}

bool StaticIMUInit::AddIMU(const IMU &imu) {
  if (init_success_) {
    return true;
  }

  if (options_.use_speed_for_static_checking && !is_static_) {
    LOG(WARNING) << "等待车辆静止";
    init_imu_deque_.clear();
    return false;
  }

  if (init_imu_deque_.empty()) {
    // 记录初始静止时间
    init_start_time_ = imu.timestamp_;
  }

  // 记入初始化队列
  init_imu_deque_.push_back(imu);

  double init_time = imu.timestamp_ - init_start_time_; // 初始化经过时间
  if (init_time > options_.init_time_seconds) {
    // 尝试初始化逻辑
    TryInit();
  }

  // 维持初始化队列长度
  while (init_imu_deque_.size() > options_.init_imu_queue_max_size) {
    init_imu_deque_.pop_front();
  }

  current_time_ = imu.timestamp_;
  return false;
}

bool StaticIMUInit::TryInit() {
  if (init_imu_deque_.size() < 10) {
    return false;
  }

  // 计算均值和方差
  Vec3d mean_gyro, mean_acce;
  // 计算陀螺仪均值和对角协方差
  // 1. gyro bias = mean_gyro
  math::ComputeMeanAndCovDiag(init_imu_deque_, mean_gyro, cov_gyro_,
                              [](const IMU &imu) { return imu.gyro_; });
  // 计算加速度计均值和对角协方差
  math::ComputeMeanAndCovDiag(init_imu_deque_, mean_acce, cov_acce_,
                              [this](const IMU &imu) { return imu.acce_; });

  LOG(INFO) << "estimate acce mean: " << mean_acce.transpose();
  LOG(INFO) << "estimate gyro mean: " << mean_gyro.transpose();
  LOG(INFO) << "estimate acce cov: " << cov_acce_.transpose();
  LOG(INFO) << "estimate gyro cov: " << cov_gyro_.transpose();

  // 以acce均值为方向，取9.8长度为重力
  // 2. get gravity
  gravity_ = -mean_acce / mean_acce.norm() * options_.gravity_norm;
  LOG(INFO) << "estimate gravity: " << gravity_.transpose();

  // 重新计算加计的协方差
  // 去掉重力的影响
  // 3. 此时 acce bias = mean_acce
  math::ComputeMeanAndCovDiag(
      init_imu_deque_, mean_acce, cov_acce_,
      [this](const IMU &imu) { return imu.acce_ + gravity_; });

  LOG(INFO) << "second estimate acce mean: " << mean_acce.transpose();
  LOG(INFO) << "second estimate acce cov: " << cov_acce_.transpose();

  // 5. 检查IMU噪声 协方差矩阵
  // cov_gyro_ 中每个元素的平方之和再开方
  LOG(INFO) << "gyro noise (cov normalized): " << cov_gyro_.norm();
  if (cov_gyro_.norm() > options_.max_static_gyro_var) {
    LOG(ERROR) << "陀螺仪测量噪声太大" << cov_gyro_.norm() << " > "
               << options_.max_static_gyro_var;
    return false;
  }

  LOG(INFO) << "acce noise (cov normalized): " << cov_acce_.norm();
  if (cov_acce_.norm() > options_.max_static_acce_var) {
    LOG(ERROR) << "加计测量噪声太大" << cov_acce_.norm() << " > "
               << options_.max_static_acce_var;
    return false;
  }

  // 估计测量噪声和零偏
  init_bg_ = mean_gyro;
  init_ba_ = mean_acce;

  LOG(INFO) << "IMU 初始化成功，初始化时间= "
            << current_time_ - init_start_time_
            << ", \nbg = " << init_bg_.transpose()
            << ", \nba = " << init_ba_.transpose()
            << ", \ngyro sq = " << cov_gyro_.transpose()
            << ", \nacce sq = " << cov_acce_.transpose()
            << ", \ngrav = " << gravity_.transpose()
            << ", \nnorm: " << gravity_.norm();
  LOG(INFO) << "mean gyro: " << mean_gyro.transpose()
            << " \nacce: " << mean_acce.transpose();
  init_success_ = true;
  return true;
}

bool StaticIMUInit::AddOdom(const Odom &odom) {
  // 判断车辆是否静止
  if (init_success_) {
    return true;
  }

  if (odom.left_pulse_ < options_.static_odom_pulse &&
      odom.right_pulse_ < options_.static_odom_pulse) {
    is_static_ = true;
  } else {
    is_static_ = false;
  }

  current_time_ = odom.timestamp_;
  return true;
}
} // namespace sad
