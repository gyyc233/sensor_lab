#include "include/orb_cv.h"
#include "pose_estimate.h"
#include <memory>
#include <opencv2/opencv.hpp>
#include <string>
#include <vector>

namespace SensorLab {
class Triansgulation : public PoseEstimation {
public:
  Triansgulation();
  ~Triansgulation();

  void initialization() override;

  void run() override;

  /// @brief input camera intrinsics
  /// @param intrinsics camera intrinsics array [fx, fy, cx, cy]
  void inputCameraIntrinsics(const std::vector<double> &intrinsics);

  /// @brief triangulation match points
  /// @param keypoint_l previous image key points
  /// @param keypoint_r current image key points
  /// @param matches points match result
  /// @param R rotation from l to r
  /// @param t translation from l to r
  /// @param points triangulation result
  void triangulation(const std::vector<cv::KeyPoint> &keypoint_l,
                     const std::vector<cv::KeyPoint> &keypoint_r,
                     const std::vector<cv::DMatch> &matches, const cv::Mat &R,
                     const cv::Mat &t, std::vector<cv::Point3d> &points);

  void checkReProject(cv::Mat &img_l, cv::Mat &img_r);

private:
  cv::Point2f pixel2cam(const cv::Point2d &p, const cv::Mat &K);

  std::vector<cv::KeyPoint> keypoint_l_;
  std::vector<cv::KeyPoint> keypoint_r_;
  std::vector<cv::DMatch> matches_;
  cv::Mat frame_rotation_;
  cv::Mat frame_translation_;

  cv::Mat camera_matrix_;

  std::vector<cv::Point3d> points_;
};
} // namespace SensorLab