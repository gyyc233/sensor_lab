#ifndef __CV_EIGEN_CONVERT_H__
#define __CV_EIGEN_CONVERT_H__

#include <Eigen/Core>
#include <Eigen/Dense>
#include <opencv2/core/eigen.hpp>
#include <opencv2/opencv.hpp>

template <typename T>
std::vector<std::vector<T>> cv_mat_convert_to_vector(cv::Mat &mat, size_t rows,
                                                     size_t cols) {}

template <typename T>
std::vector<std::vector<T>> eigen_mat_convert_to_vector(
    Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> &mat, size_t rows,
    size_t cols) {}

template <typename T>
cv::Mat vector_convert_to_cv_mat(std::vector<std::vector<T>> &mat, size_t rows,
                                 size_t cols) {}

template <typename T>
Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic>
vector_convert_to_eigen(std::vector<std::vector<T>> &mat, size_t rows,
                        size_t cols) {}

#endif
