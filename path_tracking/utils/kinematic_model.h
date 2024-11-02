#ifndef PATH_TRACKING_KINEMATIC_MODEL_H
#define PATH_TRACKING_KINEMATIC_MODEL_H
#include <Eigen/Dense>
#include <cmath>
#include <iostream>
#include <vector>

using namespace std;
using namespace Eigen;

// 移动机器人运动学模型（以后轴为车辆中心的单车运动模型）
class KinematicModel {
public:
  double x;
  double y;
  double psi; // 车身方向与水平轴夹角，也叫航向角
  double v;
  double L;  // 前后轴距
  double dt; // 采样时间

public:
  KinematicModel();

  /// @brief 机器人运动学模型
  /// @param x 位置x
  /// @param y 位置y
  /// @param psi 车身方向与水平轴夹角
  /// @param v 速度
  /// @param l 前后轴距
  /// @param dt 采样时间
  KinematicModel(double x, double y, double psi, double v, double l, double dt);

  vector<double> getState();

  /// @brief 控制量为转向角delta_f和加速度a
  /// @param accel
  /// @param delta_f
  void updateState(double accel, double delta_f);

  /// @brief 将模型离散化后的状态空间矩阵
  /// @param ref_delta 名义控制输入
  /// @param ref_yaw 车辆与水平轴夹角，偏航角
  /// @return
  vector<MatrixXd> stateSpace(double ref_delta, double ref_yaw);
};

#endif // PATH_TRACKING_KINEMATIC_MODEL_H
