#include "ekf_sample.h"
#include <iostream>

EKF::EKF() {}

EKF::~EKF() {}

EKF::EKF(const Eigen::MatrixXd &A_k,
         const Eigen::VectorXd &process_noise_v_k_minus_1,
         const Eigen::MatrixXd &Q_k, const Eigen::MatrixXd &H_k,
         const Eigen::MatrixXd &R_k, const Eigen::VectorXd &sensor_noise_w_k)
    : A_k_(A_k), process_noise_v_k_minus_1_(process_noise_v_k_minus_1),
      Q_k_(Q_k), H_k_(H_k), R_k_(R_k), sensor_noise_w_k_(sensor_noise_w_k),
      m_(H_k.rows()), n_(A_k.rows()) {}

Eigen::MatrixXd EKF::getB(double yaw_rad, double delta_k) {
  Eigen::MatrixXd B(3, 2);
  B << cos(yaw_rad) * delta_k, 0, sin(yaw_rad) * delta_k, 0, 0, delta_k;
  return B;
}

void EKF::ekf_process(Eigen::VectorXd &z_k_observation_vector,
                      Eigen::VectorXd &state_estimate_k_minus_1,
                      Eigen::VectorXd &control_vector_k_minus_1,
                      Eigen::MatrixXd &P_k_minus_1, double dt,
                      Eigen::VectorXd &state_estimate_k, Eigen::MatrixXd &P_k) {
  // =================== predicate ==================
  // 1. Predict the state estimate at time k based on the state
  // estimate at time k-1 and the control input applied at time k-1.
  state_estimate_k =
      A_k_ * state_estimate_k_minus_1 +
      getB(state_estimate_k_minus_1[2], dt) * control_vector_k_minus_1 +
      process_noise_v_k_minus_1_;
  std::cout << "State Estimate Before EKF = \n" << state_estimate_k << std::endl;

  // 2. Predict the state covariance estimate based on the previous
  // covariance and some noise
  // here jacobian1 is identify matrix
  P_k = A_k_ * P_k_minus_1 * A_k_.transpose() + Q_k_;

  // =================== correct ==================
  // 3. Calculate the difference between the actual sensor measurements
  // at time k minus what the measurement model predicted
  // the sensor measurements would be for the current time step k.
  auto measurement_residual_y_k =
      z_k_observation_vector - (H_k_ * state_estimate_k + sensor_noise_w_k_);
  std::cout << "Observation = \n" << z_k_observation_vector << std::endl;

  // 3. Calculate the measurement residual covariance
  auto S_K = H_k_ * P_k * H_k_.transpose() + R_k_;

  // 3. Calculate the near-optimal Kalman gain
  // We use pseudoinverse since some of the matrices might be
  // non-square or singular.
  auto K_k = P_k * H_k_.transpose() * S_K.inverse();
  std::cout << "K_k = \n" << K_k << std::endl;

  // 4. Calculate an correct state estimate for time k
  state_estimate_k = state_estimate_k + K_k * measurement_residual_y_k;
  std::cout << "corrected state_estimate_k = \n" << state_estimate_k << std::endl;

  // 5. correct the state covariance estimate for time k
  P_k = P_k - K_k * H_k_ * P_k;
  std::cout << "corrected P_k = \n" << P_k << std::endl;
}
