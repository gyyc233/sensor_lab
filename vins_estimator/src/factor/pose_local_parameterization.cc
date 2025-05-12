#include "pose_local_parameterization.h"

namespace sensor_lab {

int PoseLocalParameterization::AmbientSize() const { return 7; }

int PoseLocalParameterization::TangentSize() const { return 6; }

bool PoseLocalParameterization::Plus(const double *x, const double *delta,
                                     double *x_plus_delta) const {
  // decompose x
  Eigen::Map<const Eigen::Vector3d> _p(x);
  Eigen::Map<const Eigen::Quaterniond> _q(x + 3);

  // decompose delta
  Eigen::Map<const Eigen::Vector3d> dp(delta);
  // Utility::deltaQ() 将旋转向量转换为四元数 dq
  Eigen::Quaterniond dq =
      Utility::deltaQ(Eigen::Map<const Eigen::Vector3d>(delta + 3));

  // decompose x_plus_delta
  Eigen::Map<Eigen::Vector3d> p(x_plus_delta);
  Eigen::Map<Eigen::Quaterniond> q(x_plus_delta + 3);

  // update x_plus_delta
  p = _p + dp;
  q = (_q * dq).normalized();

  return true;
}

bool PoseLocalParameterization::Minus(const double *y, const double *x,
                                      double *y_minus_x) const {
  Eigen::Map<const Eigen::Vector3d> p_y(y);
  Eigen::Map<const Eigen::Quaterniond> q_y(y + 3);

  Eigen::Map<const Eigen::Vector3d> p_x(x);
  Eigen::Map<const Eigen::Quaterniond> q_x(x + 3);

  Eigen::Map<Eigen::Vector3d> p_y_minus_x(y_minus_x);
  Eigen::Map<Eigen::Vector3d> q_y_minus_x(y_minus_x + 3);

  p_y_minus_x = p_y - p_x;
  // 四元数 -> 李代数
  Eigen::AngleAxisd axisd((q_x.inverse() * q_y).normalized());
  q_y_minus_x = axisd.angle() * axisd.axis();

  return true;
}

bool PoseLocalParameterization::PlusJacobian(const double *x,
                                             double *jacobian) const {
  // 每一行对应一个全局变量（7个：tx, ty, tz, qx, qy, qz, qw）
  // 每一列对应一个局部更新维度（6个：dx, dy, dz, rx, ry, rz）
  Eigen::Map<Eigen::Matrix<double, 7, 6, Eigen::RowMajor>> j(jacobian);
  j.topRows<6>().setIdentity();
  j.bottomRows<1>().setZero();

  // 此雅可比矩阵是一个简化的近似版本，忽略了旋转更新对四元数其他分量的非线性影响
  // 在大多数 VIO/SLAM 应用中，这种近似是可接受的，尤其是在增量较小时
  // 如果需要更精确的导数，可以基于李代数 $ se(3) $
  // 推导完整的右扰动或左扰动雅可比

  return true;
}

bool PoseLocalParameterization::MinusJacobian(const double *x,
                                              double *jacobian) const {
  Eigen::Map<Eigen::Matrix<double, 6, 7, Eigen::RowMajor>> jaco(jacobian);

  jaco.rightCols<6>().setIdentity();
  jaco.leftCols<1>().setZero();

  return true;
}

} // namespace sensor_lab
