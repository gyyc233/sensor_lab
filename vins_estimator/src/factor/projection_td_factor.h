#ifndef VINS_ESTIMATOR_FACTOR_PROJECTION_TD_FACTOR
#define VINS_ESTIMATOR_FACTOR_PROJECTION_TD_FACTOR

#include "../parameters.h"
#include "../utility/utility.h"
#include "tic_toc.h"
#include <Eigen/Dense>
#include <ceres/ceres.h>

namespace sensor_lab {
// 构建一个带时间偏移（Time Delay）的视觉重投影误差项
// 支持对 时间偏移（Time Delay） 和 滚动快门（Rolling Shutter） 进行建模

// 2维残差，5组残差块：i帧位姿7维，j帧位姿7维，imu-camera外参7维，i帧时间偏移1维，j帧时间偏移1维
class ProjectionTdFactor : public ceres::SizedCostFunction<2, 7, 7, 7, 1, 1> {
public:
  // _pts_i, _pts_j: 图像帧 i 和 j 上的归一化相机坐标（去畸变后）
  // _velocity_i, _velocity_j: 对应帧下的像素速度（可选）
  // _td_i, _td_j: 时间偏移值（IMU 与图像之间的延迟）
  ProjectionTdFactor(const Eigen::Vector3d &_pts_i,
                     const Eigen::Vector3d &_pts_j,
                     const Eigen::Vector2d &_velocity_i,
                     const Eigen::Vector2d &_velocity_j, const double _td_i,
                     const double _td_j, const double _row_i,
                     const double _row_j);

  // 计算残差以及其相对几个参数块的jacobian
  virtual bool Evaluate(double const *const *parameters, double *residuals,
                        double **jacobians) const;

  void check(double **parameters);

  Eigen::Vector3d pts_i, pts_j; // 特征点在帧i和j上的归一化相机坐标
  Eigen::Vector3d velocity_i, velocity_j; // 特征点在帧i和j上的像素速度
  double td_i, td_j; // 帧i和j时刻，imu-camera之间的时间偏移
  Eigen::Matrix<double, 2, 3> tangent_base; // 切平面基底
  double row_i, row_j;              // 图像行号，用于滚动快门建模
  static Eigen::Matrix2d sqrt_info; // 信息矩阵的平方根
  static double sum_t;              // 统计 Evaluate() 函数的执行时间
};
} // namespace sensor_lab

#endif
