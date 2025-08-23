#include "imu_preintegration.h"
#include <glog/logging.h>

namespace sad {

IMUPreintegration::IMUPreintegration(Options options) {
  bg_ = options.init_bg_;
  ba_ = options.init_ba_;
  const float ng2 = options.noise_gyro_ * options.noise_gyro_;
  const float na2 = options.noise_acce_ * options.noise_acce_;
  noise_gyro_acce_.diagonal() << ng2, ng2, ng2, na2, na2, na2;
}

void IMUPreintegration::Integrate(const IMU &imu, double dt) {
  // 去掉零偏的测量
  Vec3d gyr = imu.gyro_ - bg_; // 陀螺
  Vec3d acc = imu.acce_ - ba_; // 加计

  // 更新dv, dp, 见(4.13), (4.16)
  // 1. 更新位置和速度的测量值(也叫观测值)
  dp_ = dp_ + dv_ * dt + 0.5f * dR_.matrix() * acc * dt * dt;
  dv_ = dv_ + dR_ * acc * dt;

  // dR先不更新，因为A, B阵还需要现在的dR

  // 运动方程雅可比矩阵系数，A,B阵，见(4.29)
  // 2. 预积分噪声模型
  // 噪声项的运动模型递推 A对应[theta, v, p] B对应[g, a]
  Eigen::Matrix<double, 9, 9> A;
  A.setIdentity();
  Eigen::Matrix<double, 9, 6> B;
  B.setZero();

  Mat3d acc_hat = SO3::hat(acc);
  double dt2 = dt * dt;

  // NOTE A, B左上角块与公式稍有不同 另外两项在后面
  A.block<3, 3>(3, 0) = -dR_.matrix() * dt * acc_hat;
  A.block<3, 3>(6, 0) = -0.5f * dR_.matrix() * acc_hat * dt2;
  A.block<3, 3>(6, 3) = dt * Mat3d::Identity();

  B.block<3, 3>(3, 3) = dR_.matrix() * dt;
  B.block<3, 3>(6, 3) = 0.5f * dR_.matrix() * dt2;

  // 更新各雅可比，见式(4.39)
  // 3. 更新观测量对零偏的各jacobian
  dP_dba_ = dP_dba_ + dV_dba_ * dt - 0.5f * dR_.matrix() * dt2; // (4.39d)
  dP_dbg_ = dP_dbg_ + dV_dbg_ * dt -
            0.5f * dR_.matrix() * dt2 * acc_hat * dR_dbg_;   // (4.39e)
  dV_dba_ = dV_dba_ - dR_.matrix() * dt;                     // (4.39b)
  dV_dbg_ = dV_dbg_ - dR_.matrix() * dt * acc_hat * dR_dbg_; // (4.39c)

  // 旋转部分
  // 4. 更新旋转部分测量值
  Vec3d omega = gyr * dt;        // 转动量
  Mat3d rightJ = jr_test(omega); // 右雅可比
  SO3 deltaR = SO3::exp(omega);  // exp后
  dR_ = dR_ * deltaR;            // (4.9)

  // 这里接着更新噪声模型递推
  A.block<3, 3>(0, 0) = deltaR.matrix().transpose();
  B.block<3, 3>(0, 0) = rightJ * dt;

  // 更新噪声项
  cov_ = A * cov_ * A.transpose() + B * noise_gyro_acce_ * B.transpose();

  // 更新dR_dbg
  dR_dbg_ = deltaR.matrix().transpose() * dR_dbg_ - rightJ * dt; // (4.39a)

  // 增量积分时间
  // 5. 更新积分时间
  dt_ += dt;
}

SO3 IMUPreintegration::GetDeltaRotation(const Vec3d &bg) {
  // 零偏更新后，基于预积分与零偏线性化重新计算旋转
  return dR_ * SO3::exp(dR_dbg_ * (bg - bg_));
}

Vec3d IMUPreintegration::GetDeltaVelocity(const Vec3d &bg, const Vec3d &ba) {
  // 零偏更新后，基于预积分与零偏线性化重新计算速度
  return dv_ + dV_dbg_ * (bg - bg_) + dV_dba_ * (ba - ba_);
}

Vec3d IMUPreintegration::GetDeltaPosition(const Vec3d &bg, const Vec3d &ba) {
  // 零偏更新后，基于预积分与零偏线性化重新计算位移
  return dp_ + dP_dbg_ * (bg - bg_) + dP_dba_ * (ba - ba_);
}

NavStated IMUPreintegration::Predict(const sad::NavStated &start,
                                     const Vec3d &grav) const {
  // 不进行优化 直接使用预积分观测量积分
  SO3 Rj = start.R_ * dR_;
  Vec3d vj = start.R_ * dv_ + start.v_ + grav * dt_;
  Vec3d pj =
      start.R_ * dp_ + start.p_ + start.v_ * dt_ + 0.5f * grav * dt_ * dt_;

  auto state = NavStated(start.timestamp_ + dt_, Rj, pj, vj);
  state.bg_ = bg_;
  state.ba_ = ba_;
  return state;
}

Eigen::Matrix3d IMUPreintegration::jr_test(const Eigen::Vector3d &omega) {
  double theta = omega.norm();
  if (theta < 1e-6) {
    return Eigen::Matrix3d::Identity();
  }

  Eigen::Vector3d a = omega;
  a.normalize();
  double sin_theta = std::sin(theta);
  double cos_theta = std::cos(theta);

  Eigen::Matrix3d ret = (sin_theta / theta) * Eigen::Matrix3d ::Identity() +
                        (1 - sin_theta / theta) * a * a.transpose() +
                        (1 - cos_theta) / theta * hat_test(a);

  return ret;
}

Eigen::Matrix3d IMUPreintegration::jr_inv_test(const Eigen::Vector3d &omega) {
  double theta = omega.norm();
  if (theta < 1e-6) {
    return Eigen::Matrix3d::Identity();
  }

  Eigen::Vector3d a = omega;
  a.normalize();

  double cot_half_theta = cos(0.5 * theta) / sin(0.5 * theta);
  Eigen::Matrix3d ret =
      0.5 * theta * cot_half_theta * Eigen::Matrix3d ::Identity() +
      (1 - 0.5 * theta * cot_half_theta) * a * a.transpose() -
      0.5 * theta * hat_test(a);

  return ret;
}

Eigen::Matrix3d IMUPreintegration::hat_test(const Eigen::Vector3d &omega) {
  Eigen::Matrix3d Omega;
  // clang-format off
  Omega <<
      0.0, -omega[2],  omega[1],
        omega[2], 0.0, -omega[0],
      -omega[1],  omega[0], 0.0;
  // clang-format on
  return Omega;
}

} // namespace sad
