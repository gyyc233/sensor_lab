#ifndef VINS_ESTIMATOR_FACTOR_PROJECTION_FACTOR
#define VINS_ESTIMATOR_FACTOR_PROJECTION_FACTOR

#include "../parameters.h"
#include "../utility/utility.h"
#include "tic_toc.h"
#include <Eigen/Dense>
#include <ceres/ceres.h>

namespace sensor_lab {
// 在 Ceres Solver 中构建视觉重投影误差项

// 2维残差，四组参数块（第一个位姿7维，第二个位姿7维，第三个imu-camera extrinsic
// 7维，时间偏移1维）
class ProjectionFactor : public ceres::SizedCostFunction<2, 7, 7, 7, 1> {
public:
  /// @brief 构造一个重投影误差因子
  /// @param _pts_i 特征点在图像帧 i 上的归一化相机坐标（去畸变后）
  /// @param _pts_j 在图像帧 j 上的归一化相机坐标
  ProjectionFactor(const Eigen::Vector3d &_pts_i,
                   const Eigen::Vector3d &_pts_j);

  /// @brief 计算当前参数下的残差和雅可比矩阵
  /// @param parameters 包含所有参数块的指针数组 parameters[0]: Pose_i (7维)
  /// parameters[1]: Pose_j (7维) parameters[2]: imu-camera extrinsic,
  /// parameters[3]: 逆深度
  /// @param residuals 输出2维残差（u 和 v 方向的像素误差）
  /// @param jacobians 如果不为 nullptr，则输出每个参数块对应的雅可比矩阵（2x7
  /// 或 2x1）
  /// @return
  virtual bool Evaluate(double const *const *parameters, double *residuals,
                        double **jacobians) const;

  /// @brief
  /// 调试用途，用于打印当前参数状态、残差、雅可比等信息，方便验证数值正确性
  /// @param parameters
  void check(double **parameters);

  // 特征点在图像帧 i 和 j 上的归一化相机坐标（去畸变后）
  Eigen::Vector3d pts_i, pts_j;

  // 切平面基底，用于将三维空间的误差投影到二维切平面上
  Eigen::Matrix<double, 2, 3> tangent_base;

  // 信息矩阵的平方根
  static Eigen::Matrix2d sqrt_info;

  // 统计 Evaluate() 函数的执行时间
  static double sum_t;
};
} // namespace sensor_lab

#endif
