#ifndef VINS_ESTIMATOR_UTILITY_H
#define VINS_ESTIMATOR_UTILITY_H

#include <cassert>
#include <cmath>
#include <cstring>
#include <eigen3/Eigen/Dense>
#include <libgen.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/stat.h>
#include <sys/types.h>

namespace sensor_lab {

class Utility {
public:
  /// @brief 小角度旋转转对应增量四元数
  /// @details
  /// 近似计算小角度旋转对应的四元数，通常适用于相邻IMU帧间的相对旋转，避免三角函数运算，仅需加减乘除，适合实时系统
  /// @tparam Derived
  /// @param theta
  /// @return 单位四元数，表示小角度旋转
  template <typename Derived>
  static Eigen::Quaternion<typename Derived::Scalar>
  deltaQ(const Eigen::MatrixBase<Derived> &theta) {
    typedef typename Derived::Scalar Scalar_t;

    Eigen::Quaternion<Scalar_t> dq;
    Eigen::Matrix<Scalar_t, 3, 1> half_theta = theta;
    half_theta /= static_cast<Scalar_t>(2.0); // delta /2

    dq.w() = static_cast<Scalar_t>(1.0); // 实部取１
    dq.x() = half_theta.x();             // 虚部取 delta /2
    dq.y() = half_theta.y();
    dq.z() = half_theta.z();

    // IMU预积分中计算相邻IMU帧间的相对旋转增量
    // 在构建IMU残差时，计算预测旋转与观测旋转的差异
    return dq;
  }

  /// @brief 计算三维向量的反对称矩阵
  /// @tparam Derived
  /// @param q
  /// @return
  template <typename Derived>
  static Eigen::Matrix<typename Derived::Scalar, 3, 3>
  skewSymmetric(const Eigen::MatrixBase<Derived> &q) {
    Eigen::Matrix<typename Derived::Scalar, 3, 3> ans;
    ans << typename Derived::Scalar(0), -q(2), q(1), q(2),
        typename Derived::Scalar(0), -q(0), -q(1), q(0),
        typename Derived::Scalar(0);
    return ans;
  }
};

} // namespace sensor_lab

#endif
