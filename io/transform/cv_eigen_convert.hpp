#ifndef __CV_EIGEN_CONVERT_H__
#define __CV_EIGEN_CONVERT_H__

#include <Eigen/Core>
#include <Eigen/Dense>
#include <iostream>
#include <opencv2/core/eigen.hpp>
#include <opencv2/opencv.hpp>
#include <vector>

template <typename T>
std::vector<std::vector<T>> cv_mat_convert_to_vector_2d(cv::Mat &mat) {
  assert(mat.channels() == 1);
  std::vector<std::vector<T>> result;
  for (int i = 0; i < mat.rows; i++) {
    T *row_ptr = mat.ptr<T>(i);
    std::vector<T> path;
    for (int j = 0; j < mat.cols; j++) {
      T *data_ptr = &row_ptr[j];
      path.push_back(*data_ptr);
    }
    result.push_back(path);
  }
  return result;
}

template <typename T>
std::vector<std::vector<T>> eigen_mat_convert_to_vector_2d(
    Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> &mat, size_t rows,
    size_t cols) {}

template <typename T>
cv::Mat vector_convert_to_cv_mat_2d(std::vector<std::vector<T>> &mat,
                                    size_t rows, size_t cols) {}

template <typename T>
Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic>
vector_convert_to_eigen_2d(std::vector<std::vector<T>> &mat) {
  Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> result;
  result.resize(mat.size(), mat[0].size());

  for (size_t r = 0; r < mat.size(); r++) {
    for (size_t c = 0; c < mat[0].size(); c++) {
      result(r, c) = mat[r][c];
    }
  }
  return result;
}

template <typename T>
Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic>
cv_mat_convert_to_eigen(cv::Mat &mat) {
  std::vector<std::vector<T>> vec = cv_mat_convert_to_vector_2d<T>(mat);
  return vector_convert_to_eigen_2d(vec);
}

#endif
