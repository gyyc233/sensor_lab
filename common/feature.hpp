#ifndef __FEATURE_HPP__
#define __FEATURE_HPP__

#include "my_time.h"
#include <cassert>
#include <cmath>
#include <iostream>
#include <vector>

template <typename Type, typename Iterator>
Type getFuture(Iterator begin, Iterator end) {
  Type ret;
  size_t num = 0;
  for (Iterator it = begin; it != end; it++) {
    ret += *it;
    num++;
  }
  assert(num != 0);

  return ret / static_cast<Type>(num);
}

template <typename Type, typename Iterator>
Type getBiasedVariance(Iterator begin, Iterator end) {
  Type sum = 0;
  Type average = getFuture<Type, Iterator>(begin, end);
  size_t num = 0;
  for (Iterator it = begin; it != end; it++) {
    sum += pow(*it - average, 2);
    num++;
  }

  assert(num != 0);
  sum = sum / static_cast<Type>(num);

  return (sum);
}

template <typename Type, typename Iterator>
Type getBiasedStandardDeviation(Iterator begin, Iterator end) {
  Type ret = getBiasedVariance<Type, Iterator>(begin, end);
  return sqrt(ret);
}

template <typename Type, typename Iterator>
Type getUnbiasedVariance(Iterator begin, Iterator end) {
  Type sum = 0;
  Type average = getFuture<Type, Iterator>(begin, end);
  size_t num = 0;
  for (Iterator it = begin; it != end; it++) {
    sum += pow(*it - average, 2);
    num++;
  }

  assert(num > 1);
  sum = sum / static_cast<Type>(num - 1);

  return (sum);
}

template <typename Type, typename Iterator>
Type getUnbiasedStandardDeviation(Iterator begin, Iterator end) {
  Type ret = getUnbiasedVariance<Type, Iterator>(begin, end);
  return sqrt(ret);
}

/// @brief estimate mean of each cols
/// @tparam T value type
/// @param mat matrix
/// @param mean mean matrix
template <typename T>
void meanOfCols(const std::vector<std::vector<T>> &mat, std::vector<T> &mean) {
  const size_t rows = mat.size();
  const size_t cols = mat[0].size();

  mean.resize(cols, (T)0);
  for (size_t c = 0; c < cols; c++) {
    for (size_t r = 0; r < rows; r++) {
      mean[c] += mat[r][c];
    }
  }

  for (auto &it : mean) {
    it = it / rows;
    std::cout << "mean of cols: " << it << std::endl;
  }
}

/// @brief estimate mean of each rows
/// @tparam T value type
/// @param mat matrix
/// @param mean mean matrix
template <typename T>
void meanOfRows(const std::vector<std::vector<T>> &mat,
                std::vector<std::vector<T>> &mean) {
  const size_t rows = mat.size();
  const size_t cols = mat[0].size();

  mean.resize(rows, std::vector<T>(1));
  for (size_t r = 0; r < rows; r++) {
    for (size_t c = 0; c < cols; c++) {
      mean[r][0] += mat[r][c];
    }
  }

  for (auto &it : mean) {
    it.front() = it.front() / cols;
    std::cout << "mean of row: " << it.front() << std::endl;
  }
}

template <typename T>
std::vector<std::vector<T>>
matrixTranspose(const std::vector<std::vector<T>> &mat) {
  const size_t rows = mat.size();
  const size_t cols = mat[0].size();

  std::vector<std::vector<T>> mat_transpose(cols, std::vector<T>(rows));
  for (size_t r = 0; r < rows; r++) {
    for (size_t c = 0; c < cols; c++) {
      mat_transpose[c][r] = mat[r][c];
      std::cout << "mat_transpose[c][r]: " << c << ", " << r << " "
                << mat_transpose[c][r] << std::endl;
    }
  }

  return mat_transpose;
}

template <typename T>
std::vector<std::vector<T>>
matrixDot(const std::vector<std::vector<T>> &left_mat,
          const std::vector<std::vector<T>> &right_mat) {
  const size_t left_r = left_mat.size();
  const size_t left_c = left_mat[0].size();

  const size_t right_r = right_mat.size();
  const size_t right_c = right_mat[0].size();
  assert(left_c == right_r);

  std::vector<std::vector<T>> dot(left_r, std::vector<T>(right_c, 0));
  for (size_t m = 0; m < dot.size(); m++) {
    for (size_t n = 0; n < dot[0].size(); n++) {
      T element = 0;
      for (size_t i = 0; i < left_c; i++) {
        element += left_mat[m][i] * right_mat[i][n];
      }

      dot[m][n] = element;
      std::cout << "dot[m][n]: " << m << ", " << n << " " << dot[m][n]
                << std::endl;
    }
  }
  return dot;
}

/// @brief calculate covariance
/// @tparam T value type
/// @param mat rows: sample number; cols: data dimension
/// @param cov output rows: data dimension; cols: data dimension
/// @param mean rows: 1; cols: mean of each data dimension
template <typename T>
void calculateCovariance(const std::vector<std::vector<T>> &mat,
                         std::vector<std::vector<T>> &cov,
                         std::vector<T> &mean) {
  CostMillisecond cost;
  const size_t rows = mat.size();    // data size
  const size_t cols = mat[0].size(); // data dimension

  // get mean of each dimension
  meanOfCols<T>(mat, mean);

  // data centerization
  std::vector<std::vector<T>> center_mat = mat;
  for (size_t r = 0; r < rows; r++) {
    for (size_t c = 0; c < cols; c++) {
      center_mat[r][c] = mat[r][c] - mean[c];
      std::cout << "center_mat[r][c]: " << r << ", " << c << " "
                << center_mat[r][c] << std::endl;
    }
  }

  // mat transpose
  std::vector<std::vector<T>> mat_transpose = matrixTranspose(center_mat);

  // estimate covariance
  std::vector<std::vector<T>> covariance_mat =
      matrixDot(mat_transpose, center_mat);
}

#endif
