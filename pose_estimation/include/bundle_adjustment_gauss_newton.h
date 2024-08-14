#include "include/orb_cv.h"
#include "pose_estimate.h"
#include "sophus/se3.h"
#include <Eigen/Core>
#include <memory>
#include <opencv2/opencv.hpp>
#include <string>
#include <vector>

namespace SensorLab {
class BA_GaussNewton : public PoseEstimation {
public:
  BA_GaussNewton();
  ~BA_GaussNewton();

  void inputParams(const char *left_img, const char *right_img,
                   const char *left_depth_img, const char *right_depth_img);

  void initialization() override;

  void run() override;

  void setIteration(int iteration_time);

  void inputCameraIntrinsics(const std::vector<double> &intrinsics);

private:
  void getWorldFramePointsAnd2DPoints();

  void bundleAdjustmentGaussNewton(
      const std::vector<Eigen::Vector3d,
                        Eigen::aligned_allocator<Eigen::Vector3d>> &points_3d,
      const std::vector<Eigen::Vector2d,
                        Eigen::aligned_allocator<Eigen::Vector2d>> &points_2d,
      const cv::Mat &K, Sophus::SE3 &pose);

  cv::Mat camera_intrinsics_mat_;
  cv::Point2d principal_point_;
  double focal_;

  std::unique_ptr<ORB_CV> feature_detect_ptr_;

  std::vector<cv::DMatch> orb_match_result_;
  std::vector<cv::KeyPoint> key_points_img_l_;
  std::vector<cv::KeyPoint> key_points_img_r_;

  cv::Mat left_image_depth_;
  cv::Mat right_image_depth_;

  std::vector<cv::Point3f> world_frame_points_;
  std::vector<cv::Point2f> camera_frame_points_;

  std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>>
      world_3d_points_;
  std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d>>
      camera_2d_points_;
  // Eigen管理内存和C++11中的方法是不一样的，所以需要单独强调元素的内存分配和管理

  int iteration_time_;
};
} // namespace SensorLab
