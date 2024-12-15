#include "imu_integration.h"

using namespace sad;

IMUIntegration::IMUIntegration(const Vec3d &gravity, const Vec3d &bg,
                               const Vec3d &ba)
    : gravity_(gravity), bg_(bg), ba_(ba) {}

void IMUIntegration::addIMU(const sad::IMU &imu) {
  double dt = imu.timestamp_ - timestamp_;

  // assume that the imu time interval is within 0 to 0.1
  if (dt > 0 && dt < 0.1) {
    // update PVQ
    p_ = p_ + v_ * dt + 0.5 * gravity_ * dt * dt +
         0.5 * (R_ * (imu.acce_ - ba_)) * dt * dt;
    v_ = v_ + R_ * (imu.acce_ - ba_) * dt + gravity_ * dt;
    R_ = R_ * Sophus::SO3::exp((imu.gyro_ - bg_) * dt);
  }

  // update timestamp
  timestamp_ = imu.timestamp_;
}

NavStated IMUIntegration::getNavState() const {
  return NavStated(timestamp_, R_, p_, v_, bg_, ba_);
}

SO3 IMUIntegration::getR() const { return R_; }
Vec3d IMUIntegration::getV() const { return v_; }
Vec3d IMUIntegration::getP() const { return p_; }
