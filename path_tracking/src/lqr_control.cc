#include "lqr_control.h"

LQRControl::LQRControl(int n) : n_(n) {}

LQRControl::~LQRControl() {}

Eigen::MatrixXd LQRControl::calRicatti(Eigen::MatrixXd &A, Eigen::MatrixXd &B,
                                       Eigen::MatrixXd &Q, Eigen::MatrixXd &R) {
  Eigen::MatrixXd P = Q;
  Eigen::MatrixXd P_new;

  for (int i = 0; i < n_; i++) {
    P_new = A.transpose() * P * A -
            A.transpose() * P * B * (R + B.transpose() * P * B).inverse() *
                B.transpose() * P * A +
            Q;
    if ((P - P_new).maxCoeff() < EPS && (P_new - P).maxCoeff() < EPS) {
      break;
    }
    P = P_new;
  }

  return P_new;
}

double LQRControl::lqrControl(std::vector<double> robot_state,
                              std::vector<std::vector<double>> refer_path,
                              double s0, Eigen::MatrixXd A, Eigen::MatrixXd B,
                              Eigen::MatrixXd Q, Eigen::MatrixXd R) {
  Eigen::MatrixXd x(3, 1);
  x << robot_state[0] - refer_path[s0][0], robot_state[1] - refer_path[s0][1],
      robot_state[2] - refer_path[s0][2];
  Eigen::MatrixXd P = calRicatti(A, B, Q, R);

  // estimate k
  Eigen::MatrixXd k =
      -(R + B.transpose() * P * B).inverse() * B.transpose() * P * A;
  // estimate u
  Eigen::MatrixXd u = k * x; // [v-ref_v, delta-ref_delta]

  return u(1, 0);
}
