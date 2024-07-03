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
  const int rows = mat.size();
  const int cols = mat[0].size();
}