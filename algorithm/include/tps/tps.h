#include <memory>
#include <opencv2/opencv.hpp>
#include <string>
#include <vector>

namespace Algorithm {

class TPSOperator {
public:
  TPSOperator();
  virtual ~TPSOperator();

  /// @brief tps basis function
  /// @param r the square of the distance between two points
  /// @return tps basis
  double tps_basis(double r);

  void calculate_K(const std::vector<cv::Point2f> &points, cv::Mat &K);

  void calculate_L(const std::vector<cv::Point2f> &points, cv::Mat &L);

  // W = {w0, w1, w2, ..., a1, ax, ay}
  // LW = Y,求W
  void calculate_W(const std::vector<cv::Point2f> &points_source,
                   const std::vector<cv::Point2f> &points_target, cv::Mat &W);

  /// @brief via tps W matrix, built map of one source point to the target point
  /// @param points_source
  /// @param source_point source image pixel
  /// @param W
  /// @return maping target point
  cv::Point2f tps_transformation(const std::vector<cv::Point2f> &points_source,
                                 const cv::Point2f &source_point,
                                 const cv::Mat &W);

  /// @brief via tps W matrix, built map of all source image pixel points to
  /// target points
  /// @param points_source source points
  /// @param W tps w MATRIX
  /// @param rows image rows
  /// @param cols image cols
  /// @param target_x all target points x coordinate
  /// @param target_y all target points y coordinate
  void tps_map(const std::vector<cv::Point2f> &points_source, const cv::Mat &W,
               const int rows, const int cols, cv::Mat &target_x,
               cv::Mat &target_y);
};

} // namespace Algorithm
