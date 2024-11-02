#include "kinematic_model.h"

KinematicModel::KinematicModel(double x, double y, double psi, double v,
                               double l, double dt)
    : x(x), y(y), psi(psi), v(v), L(l), dt(dt) {}

void KinematicModel::updateState(double accel, double delta_f) {
  x = x + v * cos(psi) * dt;
  y = y + v * sin(psi) * dt;
  psi = psi + v / L * tan(delta_f) * dt;
  v = v + accel * dt;
}

vector<double> KinematicModel::getState() { return {x, y, psi, v}; }

vector<MatrixXd> KinematicModel::stateSpace(double ref_delta, double ref_yaw) {
  MatrixXd A(3, 3), B(3, 2);
  // A是状态空间表达式对x,y,车身与水平轴夹角的jacobi矩阵
  A << 1.0, 0.0, -v * dt * sin(ref_yaw), 0.0, 1.0, v * dt * cos(ref_yaw), 0.0,
      0.0, 1.0;

  // B是状态空间表达式对速度，轮子转向角的jacobi矩阵
  B << dt * cos(ref_yaw), 0, dt * sin(ref_yaw), 0, dt * tan(ref_delta) / L,
      v * dt / (L * cos(ref_delta) * cos(ref_delta));
  return {A, B};
}
