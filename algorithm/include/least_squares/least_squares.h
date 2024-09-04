#pragma once

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <opencv2/opencv.hpp>
#include <vector>

namespace Algorithm {
class LeastSquare {
public:
  LeastSquare() = delete;
  LeastSquare(const int order);
  ~LeastSquare();

  void inputX(const std::vector<double> &x);

  void inputY(const std::vector<double> &y);

  /// @brief 代数求解最小二乘多项式拟合
  void solveViaAlgebraic();

  void solveViaMatrix();

private:
  std::vector<double> x_;
  std::vector<double> y_;

  int m_; // data size
  int n_; // order

  cv::Mat X_;
  cv::Mat coefficient_;
  cv::Mat Y_;

  cv::Mat mat_X_;
  cv::Mat mat_coefficient_opencv_;
  cv::Mat mat_coefficient_;
  cv::Mat mat_Y_;
};
}; // namespace Algorithm
