#ifndef __H_ICP_3D3D_H__
#define __H_ICP_3D3D_H__

#include "include/orb_cv.h"
#include <Eigen/Core>
#include <memory>
#include <opencv2/opencv.hpp>
#include <string>
#include <vector>

namespace SensorLab {
class ICP_3D3D {
public:
  ICP_3D3D();
  ~ICP_3D3D();

  void inputParams(const char *source_img, const char *target_img,
                   const char *source_depth_img, const char *target_depth_img);

  void inputCameraIntrinsics(const std::vector<double> &intrinsics,
                             double depth_scale);

  /// @brief get 3d points data
  /// @note target_points = R * source_points + T
  /// @param source_points
  /// @param target_points
  void getPointsData(std::vector<cv::Point3f> &source_points,
                     std::vector<cv::Point3f> &target_points);

  /// @brief ICP for 3D points pair
  /// @note target_points = R * source_points + T
  /// @param points_source
  /// @param points_target
  /// @param rotation rotation matrix
  /// @param translation translate
  void icpPoseEstimation_3D3D(const std::vector<cv::Point3f> &points_source,
                              const std::vector<cv::Point3f> &points_target,
                              cv::Mat &R, cv::Mat &t);

  void run();

  void output(cv::Mat &rotation, cv::Mat &translation);

private:
  void normalization(std::vector<cv::Point3f> &points,
                     cv::Point3f &center_point);

  std::unique_ptr<ORB_CV> feature_detect_ptr_;
  std::vector<cv::DMatch> orb_match_result_;
  std::vector<cv::KeyPoint> key_points_img_l_;
  std::vector<cv::KeyPoint> key_points_img_r_;

  // target_points = R * source_points + T
  cv::Mat source_image_depth_;
  cv::Mat target_image_depth_;

  std::vector<cv::Point3f> source_points_;
  std::vector<cv::Point3f> target_points_;

  cv::Mat camera_intrinsics_mat_;
  double depth_scale_;

  cv::Mat rotation_;
  cv::Mat translation_;
};
} // namespace SensorLab

#endif
