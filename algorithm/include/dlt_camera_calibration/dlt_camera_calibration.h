#include <Eigen/Dense>
#include <Eigen/QR>
#include <Eigen/SVD>
#include <opencv2/opencv.hpp>
#include <string>
#include <vector>

namespace Algorithm {

class DltCameraCalibrationOperator {

public:
  DltCameraCalibrationOperator();

  ~DltCameraCalibrationOperator();

  void setWorldPoints(const std::vector<cv::Point3d> &world_points);

  void setImagePoints(const std::vector<cv::Point2d> &image_points);

  int builtLinearEquations();

  int solve();

private:
  std::vector<cv::Point3d> world_points_;

  std::vector<cv::Point2d> image_points_;

  cv::Mat linear_equation_mat_;

  cv::Mat project_mat_;
};

}; // namespace Algorithm
