#ifndef __DISCRETE_KALMAN_FILTER_H__
#define __DISCRETE_KALMAN_FILTER_H__

#include <Eigen/Dense>

class DiscreteKalmanFilter {
public:
  /// @brief Create a Discrete Kalman filter with the specified matrices.
  /// 离散卡尔曼
  /// @param dt Discrete time step
  /// @param A System dynamics matrix (状态方程 x_k=A * x_(k-1) + B * u_k-1
  /// 系数)
  /// @param H Output matrix (观测方程 z_k=H*x_k+v_k 系数)
  /// @param Q Process noise covariance 过程噪声协方差
  /// @param R Measurement noise covariance 观测噪声协方差，与传感器有关
  /// @param P Estimate error
  /// covariance　误差协方差，可以设置大概初值，跟随每次预测更新，会迭代收敛
  DiscreteKalmanFilter(double dt, const Eigen::MatrixXd &A,
                       const Eigen::MatrixXd &H, const Eigen::MatrixXd &Q,
                       const Eigen::MatrixXd &R, const Eigen::MatrixXd &P);

  /// @brief Create a blank estimator
  DiscreteKalmanFilter();

  ~DiscreteKalmanFilter();

  /// @brief Initialize the filter with initial states as zero.
  void init();

  /// @brief Initialize the filter with a guess for initial states.
  /// @param t0 initial time step
  /// @param x0 initial states
  void init(double t0, const Eigen::VectorXd &x0);

  /// @brief Update the estimated state based on measured values.
  /// @details time step is assumed to remain constant.
  /// @param y measured values
  void update(const Eigen::VectorXd &y);

  /// @brief Update the estimated state based on measured values, using the
  /// given time step and dynamics matrix.
  /// @param y
  /// @param dt
  /// @param A
  void update(const Eigen::VectorXd &y, double dt, const Eigen::MatrixXd A);

  /// @brief Return the current state
  /// @return
  Eigen::VectorXd state();

  /// @brief Return the current time
  /// @return
  double time();

private:
  // Matrices for computation
  Eigen::MatrixXd A_, H_, Q_, R_, P_, K_, P0_;

  // System dimensions
  int m_; // Number of measurements

  int n_; // Number of states

  // Initial and current time
  double t0_, t_;

  // Discrete time step
  double dt_;

  // Is the filter initialized?
  bool initialized_;

  // n-size identity
  Eigen::MatrixXd I_;

  // Estimated states
  Eigen::VectorXd x_hat_, x_hat_new_;
};

#endif
