// rewritten from
// https://automaticaddison.com/extended-kalman-filter-ekf-with-python-code-example/

#ifndef __EXTENDED_KALMAN_FILTER_H__
#define __EXTENDED_KALMAN_FILTER_H__

#include <Eigen/Dense>

class EKF {
public:
  EKF();
  ~EKF();
  EKF(const Eigen::MatrixXd &A_k,
      const Eigen::VectorXd &process_noise_v_k_minus_1,
      const Eigen::MatrixXd &Q_k, const Eigen::MatrixXd &H_k,
      const Eigen::MatrixXd &R_k, const Eigen::VectorXd &sensor_noise_w_k);

  /// @brief Extended Kalman Filter. Fuses noisy sensor measurement to create an
  /// optimal estimate of the state of the robotic system.
  /// @param z_k_observation_vector The observation from the Odometry 3x1 Array
  /// [x,y,yaw] in the global reference frame in [meters,meters,radians].
  /// @param state_estimate_k_minus_1 The state estimate at time k-1 3x1 Array
  /// [x,y,yaw] in the global reference frame in [meters,meters,radians].
  /// @param control_vector_k_minus_1 The control vector applied at time k-1 3x1
  /// NumPy Array [v,v,yaw rate] in the global reference frame in [meters per
  /// second,meters per second,radians per second].
  /// @param P_k_minus_1 The state covariance matrix estimate at time k-1 3x3
  /// Array
  /// @param dt Time interval in seconds
  /// @param state_estimate_k output near-optimal state estimate at time k 3x1
  /// Array ---> [meters,meters,radians]
  /// @param P_k state covariance_estimate for time k 3*3 array
  void ekf_process(Eigen::VectorXd &z_k_observation_vector,
                   Eigen::VectorXd &state_estimate_k_minus_1,
                   Eigen::VectorXd &control_vector_k_minus_1,
                   Eigen::MatrixXd &P_k_minus_1, double dt,
                   Eigen::VectorXd &state_estimate_k, Eigen::MatrixXd &P_k);

  /// @brief Calculates and returns the B matrix
  /// @note The control inputs are the forward speed and the rotation rate
  /// around the z axis from the x-axis in the counterclockwise direction.
  /// [v,yaw_rate] Expresses how the state of the system [x,y,yaw] changes from
  /// k-1 to k due to the control commands (i.e. control input).
  /// @param yaw_rad The yaw angle (rotation angle around the z axis) in rad
  /// @param delta_k The change in time from time step k-1 to k in sec
  /// @return 3x2 matix -> number of states x number of control inputs
  Eigen::MatrixXd getB(double yaw_rad, double delta_k);

private:
  // 3x3 matrix -> number of states x number of states matrix
  // Expresses how the state of the system [x,y,yaw] changes
  // from k-1 to k when no control command is executed.
  // Typically a robot on wheels only drives when the wheels are told to turn.
  // For this case, A is the identity matrix.
  // A is sometimes F in the literature.
  Eigen::MatrixXd A_k_;

  // Noise applied to the forward kinematics (calculation
  // of the estimated state at time k from the state
  // transition model of the mobile robot). This is a vector
  // with the number of elements equal to the number of states
  Eigen::VectorXd process_noise_v_k_minus_1_;

  // State model noise covariance matrix Q_k
  // When Q is large, the Kalman Filter tracks large changes in
  // the sensor measurements more closely than for smaller Q.
  // Q is a square matrix that has the same number of rows as states.
  Eigen::MatrixXd Q_k_;

  // Measurement matrix H_k
  // Used to convert the predicted state estimate at time k
  // into predicted sensor measurements at time k.
  // In this case, H will be the identity matrix since the
  // estimated state maps directly to state measurements from the
  // odometry data [x, y, yaw]
  // H has the same number of rows as sensor measurements
  // and same number of columns as states.
  Eigen::MatrixXd H_k_;

  // Sensor measurement noise covariance matrix R_k
  // Has the same number of rows and columns as sensor measurements.
  // If we are sure about the measurements, R will be near zero.
  Eigen::MatrixXd R_k_;

  // Sensor noise. This is a vector with the
  // number of elements equal to the number of sensor measurements.
  Eigen::VectorXd sensor_noise_w_k_;

  // System dimensions
  int m_; // Number of measurements

  int n_; // Number of states
};
#endif
