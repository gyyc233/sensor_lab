#include "discrete_kalman_filter.h"
#include <iostream>
#include <stdexcept>

DiscreteKalmanFilter::DiscreteKalmanFilter(double dt, const Eigen::MatrixXd &A,
                                           const Eigen::MatrixXd &H,
                                           const Eigen::MatrixXd &Q,
                                           const Eigen::MatrixXd &R,
                                           const Eigen::MatrixXd &P)
    : A_(A), H_(H), Q_(Q), R_(R), P0_(P), m_(H.rows()), n_(A.rows()), dt_(dt),
      initialized_(false), I_(n_, n_), x_hat_(n_), x_hat_new_(n_) {
  I_ = Eigen::MatrixXd::Identity(n_, n_);
}

DiscreteKalmanFilter::DiscreteKalmanFilter() {}

DiscreteKalmanFilter::~DiscreteKalmanFilter() {}

void DiscreteKalmanFilter::init(double t0, const Eigen::VectorXd &x0) {
  x_hat_ = x0;
  P_ = P0_;
  this->t0_ = t0;
  t_ = t0;
  initialized_ = true;
}

void DiscreteKalmanFilter::init() {
  x_hat_.setZero();
  P_ = P0_;
  t0_ = 0;
  t_ = t0_;
  initialized_ = true;
}

void DiscreteKalmanFilter::update(const Eigen::VectorXd &y) {
  if (!initialized_) {
    throw std::runtime_error("filter is not initialization");
  }

  // time predicate
  // 1. 状态估计预测 project the state ahead
  x_hat_new_ = A_ * x_hat_; // here there isn't B*u_k-1
  std::cout << "predicate x_hat_new_: " << x_hat_new_ << std::endl;

  // 2. 误差协方差估计 project the error covariance ahead
  P_ = A_ * P_ * A_.transpose() + Q_;
  std::cout << "predicate P_: " << P_ << std::endl;

  // measured correct
  // 3. 计算卡尔曼增益 compute the kalman gain
  K_ = P_ * H_.transpose() * (H_ * P_ * H_.transpose() + R_).inverse();
  std::cout << "correct K_: " << K_ << std::endl;

  // 4. 由测量值更新状态值 update estimate with measurement value
  x_hat_new_ = x_hat_new_ + K_ * (y - H_ * x_hat_new_);
  std::cout << "correct x_hat_new_: " << x_hat_new_ << std::endl;

  // 5. 误差协方差更新 update the error covariance
  P_ = (I_ - K_ * H_) * P_;
  std::cout << "correct P_: " << P_ << std::endl;

  // update x_(k-1)
  x_hat_ = x_hat_new_;
  t_ += dt_;
}

void DiscreteKalmanFilter::update(const Eigen::VectorXd &y, double dt,
                                  const Eigen::MatrixXd A) {
  this->A_ = A;
  this->dt_ = dt;
  update(y);
}

Eigen::VectorXd DiscreteKalmanFilter::state() { return x_hat_; }

double DiscreteKalmanFilter::time() { return t_; }
