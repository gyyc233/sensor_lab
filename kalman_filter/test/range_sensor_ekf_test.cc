#include "ekf.h"
#include "matplotlibcpp.h"
#include <Eigen/Dense>
#include <iostream>
#include <memory>
#include <vector>

// Extended Kalman Filter example (two-wheeled mobile robot)

int main() {
  int n = 3;       // Number of states
  int m = 3;       // Number of measurements
  double dt = 1.0; // Time step

  // Create a list of sensor observations at successive time steps
  // Each list within z_k is an observation vector. [x,y,theta]
  std::vector<std::vector<double>> z_ks;
  z_ks.push_back({4.721, 0.143, 0.006});
  z_ks.push_back({9.353, 0.284, 0.007});
  z_ks.push_back({14.773, 0.422, 0.009});
  z_ks.push_back({18.246, 0.555, 0.011});
  z_ks.push_back({22.609, 0.715, 0.012});

  // The estimated state vector at time k-1 in the global reference frame.
  // [x_k_minus_1, y_k_minus_1, yaw_k_minus_1]
  // [meters, meters, radians]
  Eigen::VectorXd state_estimate_k_minus_1(m);
  state_estimate_k_minus_1 << 0.0, 0.0, 0.0;

  // The control input vector at time k-1 in the global reference frame.
  // [v, yaw_rate]
  // [meters/second, radians/second]
  // In the literature, this is commonly u.
  // Because there is no angular velocity and the robot begins at the
  // origin with a 0 radians yaw angle, this robot is traveling along
  // the positive x-axis in the global reference frame.
  Eigen::VectorXd control_vector_k_minus_1(2);
  control_vector_k_minus_1 << 4.5, 0.0;

  Eigen::MatrixXd P_k_minus_1(n, n); // Estimate error covariance
  P_k_minus_1 << .1, 0, 0, 0, .1, 0, 0, 0, .1;

  Eigen::MatrixXd A_k(n, n);
  Eigen::VectorXd process_noise_v_k_minus_1(n);
  Eigen::MatrixXd Q_k(n, n);
  Eigen::MatrixXd H_k(m, n);
  Eigen::MatrixXd R_k(m, m);
  Eigen::VectorXd sensor_noise_w_k(m);

  A_k << 1.0, 0, 0, 0, 1.0, 0, 0, 0, 1.0;

  process_noise_v_k_minus_1 << 0.01, 0.01, 0.03;

  Q_k << 1.0, 0, 0, 0, 1.0, 0, 0, 0, 1.0;

  H_k << 1.0, 0, 0, 0, 1.0, 0, 0, 0, 1.0;

  R_k << 1.0, 0, 0, 0, 1.0, 0, 0, 0, 1.0;

  sensor_noise_w_k << 0.07, 0.07, 0.04;

  Eigen::VectorXd state_estimate_k;
  Eigen::MatrixXd P_k;

  EKF ekf_operator(A_k, process_noise_v_k_minus_1, Q_k, H_k, R_k,
                   sensor_noise_w_k);

  for (size_t i = 0; i < z_ks.size(); i++) {
    std::cout<<"\n\n========== loop index: "<<i<<"=========="<<std::endl;
    Eigen::VectorXd z_k(n);
    z_k << z_ks[i][0], z_ks[i][1], z_ks[i][2];

    ekf_operator.ekf_process(z_k, state_estimate_k_minus_1,
                             control_vector_k_minus_1, P_k_minus_1, dt,
                             state_estimate_k, P_k);

    state_estimate_k_minus_1 = state_estimate_k;
    P_k_minus_1 = P_k;
  }

  return 0;
}
