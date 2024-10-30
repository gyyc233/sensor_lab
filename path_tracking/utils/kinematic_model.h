#ifndef PATH_TRACKING_KINEMATIC_MODEL_H
#define PATH_TRACKING_KINEMATIC_MODEL_H
#include <Eigen/Dense>
#include <cmath>
#include <iostream>
#include <vector>

using namespace std;
using namespace Eigen;

// 移动机器人运动学模型（单车模型）
class KinematicModel {
public:
  double x, y, psi, v, L, dt;

public:
  KinematicModel();

  /// @brief 机器人运动学模型
  /// @param x 位置x
  /// @param y 位置y
  /// @param psi 偏航角
  /// @param v 速度
  /// @param l 轴距
  /// @param dt 采样时间
  KinematicModel(double x, double y, double psi, double v, double l, double dt);

  vector<double> getState();

  /// @brief 控制量为转向角delta_f和加速度a
  /// @param accel
  /// @param delta_f
  void updateState(double accel, double delta_f);

  /// @brief 将模型离散化后的状态空间表达
  /// @param ref_delta 名义控制输入
  /// @param ref_yaw 名义偏航角
  /// @return
  vector<MatrixXd> stateSpace(double ref_delta, double ref_yaw);
};

#endif // PATH_TRACKING_KINEMATIC_MODEL_H
