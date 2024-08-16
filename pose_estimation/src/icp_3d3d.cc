#include "icp_3d3d.h"
#include "transform/calibration_convert.hpp"

using namespace SensorLab;

ICP_3D3D::ICP_3D3D() {
  feature_detect_ptr_ = std::make_unique<ORB_CV>();
  depth_scale_ = 1;
}

ICP_3D3D::~ICP_3D3D() {}

void ICP_3D3D::inputParams(const char *source_img, const char *target_img,
                           const char *source_depth_img,
                           const char *target_depth_img) {
  feature_detect_ptr_->inputParams(source_img, target_img);
  feature_detect_ptr_->run();

  orb_match_result_.clear();
  key_points_img_l_.clear();
  key_points_img_r_.clear();
  feature_detect_ptr_->output(orb_match_result_, key_points_img_l_,
                              key_points_img_r_);

  // 16 unsigned depth image
  source_image_depth_ = cv::imread(source_depth_img, cv::IMREAD_UNCHANGED);
  target_image_depth_ = cv::imread(target_depth_img, cv::IMREAD_UNCHANGED);
}

void ICP_3D3D::inputCameraIntrinsics(const std::vector<double> &intrinsics,
                                     double depth_scale) {
  assert(intrinsics.size() == 4);
  camera_intrinsics_mat_ =
      (cv::Mat_<double>(3, 3) << intrinsics[0], 0, intrinsics[2], 0,
       intrinsics[1], intrinsics[3], 0, 0, 1);
  depth_scale_ = depth_scale;
}

void ICP_3D3D::getPointsData(std::vector<cv::Point3f> &source_points,
                             std::vector<cv::Point3f> &target_points) {
  source_points = source_points_;
  target_points = target_points_;
}

void ICP_3D3D::icpPoseEstimation_3D3D(
    const std::vector<cv::Point3f> &points_source,
    const std::vector<cv::Point3f> &points_target, cv::Mat &rotation,
    cv::Mat &translation) {}

void ICP_3D3D::normalization(std::vector<cv::Point3f> &points) {
  cv::Point3f center(0, 0, 0);
  for (size_t i = 0; i < points.size(); i++) {
    center += points[i];
  }
  center = cv::Point3f(cv::Vec3f(center) / static_cast<int>(points.size()));
}
