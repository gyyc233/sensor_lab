#include "include/orb_cv.h"
#include "pose_estimate.h"
#include <memory>
#include <opencv2/opencv.hpp>
#include <string>
#include <vector>

namespace SensorLab {
class EpipolarConstraint : public PoseEstimation {
public:
  EpipolarConstraint();
  ~EpipolarConstraint();

  void inputParams(const char *left_img, const char *right_img);

  /// @brief input camera intrinsics
  /// @param intrinsics camera intrinsics array [fx, fy, cx, cy]
  /// @param focal camera focal
  void inputCameraIntrinsics(const std::vector<double> &intrinsics,
                             double focal);

  void inputPrincipalPoints(double u, double v);

  void initialization() override;

  void run() override;

  void output(std::vector<double> &quaternion,
              std::vector<double> &translation);

private:
  void pose_estimation_2d2d(std::vector<cv::KeyPoint> &key_points_l,
                            std::vector<cv::KeyPoint> &key_points_r,
                            std::vector<cv::DMatch> &matches, cv::Mat &R,
                            cv::Mat &t);

  std::vector<cv::DMatch> orb_match_result_;
  std::vector<cv::KeyPoint> key_points_img_l_;
  std::vector<cv::KeyPoint> key_points_img_r_;

  cv::Mat camera_intrinsics_mat_;
  cv::Point2d principal_point_;
  double focal_;

  std::unique_ptr<ORB_CV> feature_detect_ptr_;
  cv::Mat rotation_;
  cv::Mat translate_;
};
} // namespace SensorLab
