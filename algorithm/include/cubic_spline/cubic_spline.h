#pragma once
#include <opencv2/opencv.hpp>
#include <vector>

namespace Algorithm {

class CubicSplineOperator {
public:
  CubicSplineOperator();

  ~CubicSplineOperator();

  /// @brief set sample x vals and y vals
  /// @param val_x sample x vals
  /// @param val_y sample y vals
  /// @return if success, return 0
  int setSamplePoints(const std::vector<double> &val_x,
                      const std::vector<double> &val_y);

  /// @brief built cubic natural spline and estimate coefficients
  /// @return if success, return 0
  int cubicSplineNatural();

  /// @brief fit vals and estimate predicate vals via cubic natural spline
  /// @param vals input vals
  /// @param predicated_vals predicated vals
  /// @return if success, return 0
  int cubicSplineFit(const std::vector<double> &vals,
                     std::vector<double> &predicated_vals);

  /// @brief check and modify data range
  /// @param vals origin data
  /// @param check_vals modify data
  void checkData(const std::vector<double> &vals,
                 std::vector<double> &check_vals);

private:
  /// @brief via H and D, calculate m and coefficients
  /// @details calculate H*m=D
  /// @param H H matrix
  /// @param D D matrix
  /// @param dx dx coefficient
  /// @param dy dy coefficients
  /// @param m estimated m matrix
  void estimateCoefMatrix(const cv::Mat &H, const cv::Mat &D, const cv::Mat &dx,
                          const cv::Mat &dy, cv::Mat &m);

  std::vector<cv::Point2f> input_points_; // input sample points
  cv::Mat a_;                             // cubic spline coef a
  cv::Mat b_;                             // cubic spline coef b
  cv::Mat c_;                             // cubic spline coef c
  cv::Mat d_;                             // cubic spline coef d

  double min_x_; // min x of sample points
  double max_x_; // max x of sample points
};

}; // namespace Algorithm