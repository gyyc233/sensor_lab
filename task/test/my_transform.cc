#include "transform/eigenGeometryTransfer.h"
#include <Eigen/Core>
#include <Eigen/Geometry>

int main() {
  euler2QuaternionFixedAxis(10, 20, 30);
  Eigen::Quaterniond quat = euler2QuaternionSelfAxis(30, 20, 10);

  euler2RotationMatrixFixedAxis(0, 0, 90);
  euler2RotationMatrixSelfAxis(30, 20, 10);

  quaternion2Euler(quat.x(), quat.y(), quat.z(), quat.w());

  auto matrix =
      quaternion2RotationMatrix(quat.x(), quat.y(), quat.z(), quat.w());
  rotationMatrix2Quaternion(matrix);
  rotationMatrix2euler(matrix);
  return 0;
}
