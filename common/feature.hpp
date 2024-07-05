#include <cassert>
#include <chrono>
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

/// @brief calculate covariance
/// @tparam T
/// @param mat rows: sample number; cols: data dimension
/// @param cov output rows: data dimension; cols: data dimension
/// @param mean rows: 1; cols: mean of each data dimension
/// @param scale
template <typename T>
void calculateCovariance(const std::vector<std::vector<T>> &mat,
                         const std::vector<std::vector<T>> &cov,
                         std::vector<T> &mean, bool scale = false) {
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
    std::cout << "it: " << it << std::endl;
  }

  // data centerization
  std::vector<std::vector<T>> center_mat = mat;
  for (size_t r = 0; r < rows; r++) {
    for (size_t c = 0; c < cols; c++) {
      center_mat[r][c] = center_mat[r][c] - mean[c];
      std::cout << "center_mat[r][c]: " << center_mat[r][c] << std::endl;
    }
  }

  // mat transpose
  std::vector<std::vector<T>> mat_transpose(cols, std::vector<T>(rows));
  for (size_t r = 0; r < rows; r++) {
    for (size_t c = 0; c < cols; c++) {
      mat_transpose[c][r] = mat[r][c];
      std::cout << "mat_transpose[c][r]: " << c << ", " << r << " "
                << mat_transpose[c][r] << std::endl;
    }
  }

  // estimate covariance
  for (size_t m = 0; m < cols; m++) {
    for (size_t n = 0; n < rows; n++) {
      // TODO:
    }
  }
}
