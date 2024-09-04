#include "least_squares/least_squares.h"
#include <glog/logging.h>

using namespace Algorithm;

LeastSquare::LeastSquare(const int order) { n_ = order; }

LeastSquare::~LeastSquare() {}

void LeastSquare::inputX(const std::vector<double> &x) {
  x_ = x;
  m_ = x_.size();

  if (m_ < n_) {
    LOG(ERROR) << "data size have to more than the order";
  }
}

void LeastSquare::inputY(const std::vector<double> &y) {
  y_ = y;
  m_ = y_.size();

  if (m_ < n_) {
    LOG(ERROR) << "data size have to more than the order";
  }
}

void LeastSquare::solveViaAlgebraic() {
  X_ = cv::Mat::zeros(n_ + 1, n_ + 1, CV_64FC1);
  Y_ = cv::Mat::zeros(n_ + 1, 1, CV_64FC1);

  // construct X
  for (int i = 0; i < n_ + 1; i++) {
    for (int j = 0; j < n_ + 1; j++) {
      for (int k = 0; k < m_; k++) {
        X_.at<double>(i, j) += std::pow(x_[k], i + j);
      }
    }
  }

  // construct Y
  for (int i = 0; i < n_ + 1; i++) {
    for (int k = 0; k < m_; k++) {
      Y_.at<double>(i, 0) += std::pow(x_[k], i) * y_[k];
    }
  }
  std::cout << "X_: \n" << X_ << std::endl;
  std::cout << "Y_: \n" << Y_ << std::endl;

  // solve
  coefficient_ = cv::Mat::zeros(n_ + 1, 1, CV_64FC1);
  if (!cv::solve(X_, Y_, coefficient_, cv::DECOMP_SVD)) {
    LOG(INFO) << "failed to solve" << std::endl;
    return;
  }
  LOG(INFO) << "algebraic coefficient:\n" << coefficient_ << std::endl;
  return;
}

void LeastSquare::solveViaMatrix() {
  mat_X_ = cv::Mat::zeros(m_, n_ + 1, CV_64FC1);
  mat_Y_ = cv::Mat::zeros(m_, 1, CV_64FC1);

  // construct X
  for (int i = 0; i < m_; i++) {
    for (int j = 0; j < n_ + 1; j++) {
      mat_X_.at<double>(i, j) = std::pow(x_[i], j);
    }
  }

  // construct Y
  for (int i = 0; i < m_ + 1; i++) {
    mat_Y_.at<double>(i, 0) += y_[i];
  }
  std::cout << "mat_X_: \n" << mat_X_ << std::endl;
  std::cout << "mat_Y_: \n" << mat_Y_ << std::endl;

  // solve
  mat_coefficient_opencv_ = cv::Mat::zeros(n_ + 1, 1, CV_64FC1);
  if (!cv::solve(mat_X_, mat_Y_, mat_coefficient_opencv_, cv::DECOMP_SVD)) {
    LOG(INFO) << "failed to opencv solve via matrix" << std::endl;
  }
  LOG(INFO) << "mat_coefficient_opencv_:\n" << mat_coefficient_opencv_;

  // A=(X^T^X)-1x=X^T^Y
  // 使用cv::solve求解(A^T * A) * coeff = A^T * B
  cv::Mat Xt = mat_X_.t();
  cv::Mat XtX = Xt * mat_X_;
  cv::Mat XtY = Xt * mat_Y_;

  if (!cv::solve(XtX, XtY, mat_coefficient_, cv::DECOMP_SVD)) {
    LOG(INFO) << "failed to opencv solve (A^T * A) * coeff = A^T * B"
              << std::endl;
  }
  LOG(INFO) << "mat_coefficient_:\n" << mat_coefficient_;
}
