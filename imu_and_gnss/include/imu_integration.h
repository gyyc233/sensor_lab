#ifndef __IMU_INTEGRATION_H__
#define __IMU_INTEGRATION_H__

#include "eigen_type/eigen_types.h"
#include "navigation_and_mapping/imu.h"
#include "navigation_and_mapping/nav_state.h"

namespace sad {
class IMUIntegration {
public:
  IMUIntegration(const Vec3d &gravity, const Vec3d &bg, const Vec3d &ba);

  void addIMU(const sad::IMU &imu);

  // get Nav state
  NavStated getNavState() const;

  SO3 getR() const;
  Vec3d getV() const;
  Vec3d getP() const;

private:
  // imu累积量
  SO3 R_;
  Vec3d v_ = Vec3d::Zero();
  Vec3d p_ = Vec3d::Zero();

  double timestamp_ = 0.0; // delta t

  // 陀螺仪和加速度计零偏
  Vec3d bg_ = Vec3d::Zero();
  Vec3d ba_ = Vec3d::Zero();

  Vec3d gravity_ = Vec3d(0, 0, -9.8);
};

} // namespace sad

#endif
