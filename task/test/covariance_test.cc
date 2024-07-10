#include "feature.hpp"
#include <Eigen/Core>
#include <Eigen/Dense>
#include <glog/logging.h>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <vector>

int main() {
  std::vector<std::vector<double>> matrix{{1.2f, 2.5f, 5.6f, -2.5f},
                                          {-3.6f, 9.2f, 0.5f, 7.2f},
                                          {4.3f, 1.3f, 9.4f, -3.4f}};

  std::vector<std::vector<double>> covariance;
  std::vector<double> mean;
  calculateCovariance<double>(matrix, covariance, mean);

  // opencv calculate covariance
  std::cout << "opencv calculate covariance" << std::endl;
  cv::Mat mat(matrix.size(), matrix[0].size(), CV_64FC1);
  for (size_t row = 0; row < matrix.size(); row++) {
    for (size_t col = 0; col < matrix[0].size(); col++) {
      mat.at<double>(row, col) = matrix[row][col];
    }
  }

  cv::Mat covar_cv, mean_cv;
  CostMillisecond *cv_cost = new CostMillisecond();
  cv::calcCovarMatrix(mat, covar_cv, mean_cv,
                      cv::COVAR_NORMAL | cv::COVAR_ROWS /* | CV_COVAR_SCALE*/,
                      CV_64FC1);
  delete cv_cost;
  LOG(INFO) << "\n" << covar_cv;
  LOG(INFO) << "\n" << mean_cv;

  // eigen calculate covariance
  std::cout << "eigen calculate covariance" << std::endl;
  std::vector<float> vec_cols;
  for (size_t i = 0; i < matrix.size(); ++i) {
    vec_cols.insert(vec_cols.begin() + i * matrix[0].size(), matrix[i].begin(),
                    matrix[i].end());
  }
  Eigen::Map<
      Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>
      m(vec_cols.data(), matrix.size(), matrix[0].size());
  LOG(INFO) << "\n" << m;

  Eigen::MatrixXf eigen_mean = m.colwise().mean();
  LOG(INFO) << "\n" << eigen_mean;

  Eigen::MatrixXf tmp(m.rows(), m.cols());
  for (int y = 0; y < m.rows(); ++y) {
    for (int x = 0; x < m.cols(); ++x) {
      tmp(y, x) = m(y, x) - eigen_mean(0, x);
    }
  }
  CostMillisecond *eigen_cost = new CostMillisecond();
  Eigen::MatrixXf covar =
      (tmp.adjoint() * tmp); // 对于实数矩阵来说，共扼矩阵与转置相同
  delete eigen_cost;
  LOG(INFO) << "eigen covariance \n" << covar;

  return 0;
}
