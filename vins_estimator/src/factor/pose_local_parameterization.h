#ifndef VINS_ESTIMATOR_FACTOR_POSE_LOCAL_PARAMETERIZATION
#define VINS_ESTIMATOR_FACTOR_POSE_LOCAL_PARAMETERIZATION

#include "../utility/utility.h"
#include <ceres/ceres.h>
#include <eigen3/Eigen/Dense>

namespace sensor_lab {

// 对于部分优化问题，尤其是sensor
// fusion的问题，优化的空间不是在Euclidean空间，而是在manifolds空间,比如旋转矩阵和四元数
// 一个三维的球形物体的切向平面是一个二维平面, 即球形物体是一个二维的manifold
// ceres里定义LocalParameterization的一个类，这个类支持自定义加法和Jocobian的计算
// 新版中ceres移除了LocalParameterization，替换为 Manifolds
// https://zhuanlan.zhihu.com/p/464661447

class PoseLocalParameterization : public ceres::Manifold {
  // 定义SE3上的加法 x_plus_delta = x + delta [qw, qx, qy, qz, tx, ty, tz]
  // delta: []  [dx, dy, dz, rx, ry, rz]
  // 前三个是平移增量，后三个是旋转向量增量（李代数形式） x: []  [qw, qx, qy,
  // qz, tx, ty, tz] 四元数和位移 使用 Utility::deltaQ() 和 Utility::deltaP()
  // 等函数完成旋转和平移的更新
  virtual bool Plus(const double *x, const double *delta,
                    double *x_plus_delta) const override;

  // 计算从局部空间（6维）到位姿全局空间（7维）的雅可比矩阵
  virtual bool PlusJacobian(const double *x, double *jacobian) const override;

  // 返回全局状态变量维度
  virtual int AmbientSize() const override;

  // 返回局部更新空间维度
  virtual int TangentSize() const override;

  // 计算两个位于 manifold 上的点之间的差值,输出到 y_minus_x
  bool Minus(const double *y, const double *x,
             double *y_minus_x) const override;

  bool MinusJacobian(const double *x, double *jacobian) const override;
};

} // namespace sensor_lab

#endif
