#include "icp_3d3d.h"
#include "transform/calibration_convert.hpp"

using namespace SensorLab;

ICP_3D3D::ICP_3D3D() {
  feature_detect_ptr_ = std::make_unique<ORB_CV>();
  feature_detect_ptr_->initialization();
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

void ICP_3D3D::run() {
  std::vector<cv::Point3f> source_points, target_points;
  for (auto m : orb_match_result_) {
    // get z
    ushort source_dep = source_image_depth_.ptr<unsigned short>(
        int(key_points_img_l_[m.queryIdx]
                .pt.y))[int(key_points_img_l_[m.queryIdx].pt.x)];

    ushort target_dep = target_image_depth_.ptr<unsigned short>(
        int(key_points_img_r_[m.trainIdx]
                .pt.y))[int(key_points_img_r_[m.trainIdx].pt.x)];

    if (source_dep == 0 || target_dep == 0) // bad depth
      continue;

    // get x, y
    cv::Point2d source_p = pixel2CameraFrame(key_points_img_l_[m.queryIdx].pt,
                                             camera_intrinsics_mat_);
    cv::Point2d target_p = pixel2CameraFrame(key_points_img_r_[m.trainIdx].pt,
                                             camera_intrinsics_mat_);
    float source_d = float(source_dep) / depth_scale_;
    float target_d = float(target_dep) / depth_scale_;
    source_points.push_back(
        cv::Point3f(source_p.x * source_d, source_p.y * source_d, source_d));
    target_points.push_back(
        cv::Point3f(target_p.x * target_d, target_p.y * target_d, target_d));
  }

  icpPoseEstimation_3D3D(source_points, target_points, rotation_, translation_);
}

void ICP_3D3D::output(cv::Mat &rotation, cv::Mat &translation) {
  rotation = rotation_;
  translation = translation_;
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
    cv::Mat &translation) {
  source_points_ = points_source;
  target_points_ = points_target;

  // 1. normalization
  cv::Point3f source_center, target_center;
  normalization(source_points_, source_center);
  normalization(target_points_, target_center);

  // 2. calculate covariance matrix w
  Eigen::Matrix3d w = Eigen::Matrix3d::Zero();
  for (size_t i = 0; i < source_points_.size(); i++) {
    w += Eigen::Vector3d(target_points_[i].x, target_points_[i].y,
                         target_points_[i].z) *
         Eigen::Vector3d(source_points_[i].x, source_points_[i].y,
                         source_points_[i].z)
             .transpose();
  }
  std::cout << "calculate covariance matrix w:\n" << w << std::endl;

  // 3. SVD on W W=U*∑*Vt
  Eigen::JacobiSVD<Eigen::Matrix3d> svd(w, Eigen::ComputeFullU |
                                               Eigen::ComputeFullV);
  Eigen::Matrix3d U = svd.matrixU();
  Eigen::Matrix3d V = svd.matrixV();
  std::cout << "U=\n" << U << std::endl;
  std::cout << "V=\n" << V << std::endl;

  // 4. R=U*Vt
  Eigen::Matrix3d R = U * (V.transpose());
  if (R.determinant() < 0) {
    R = -R;
  }

  // 5. T= target_center - R*source_center
  Eigen::Vector3d t =
      Eigen::Vector3d(target_center.x, target_center.y, target_center.z) -
      R * Eigen::Vector3d(source_center.x, source_center.y, source_center.z);

  rotation = (cv::Mat_<double>(3, 3) << R(0, 0), R(0, 1), R(0, 2), R(1, 0),
              R(1, 1), R(1, 2), R(2, 0), R(2, 1), R(2, 2));

  translation = (cv::Mat_<double>(3, 1) << t(0, 0), t(1, 0), t(2, 0));

  rotation_ = rotation;
  translation_ = translation;
  std::cout << "rotation:\n" << rotation_ << std::endl;
  std::cout << "translation:\n" << translation_ << std::endl;
}

void ICP_3D3D::normalization(std::vector<cv::Point3f> &points,
                             cv::Point3f &center_point) {
  cv::Point3f center(0, 0, 0);
  for (size_t i = 0; i < points.size(); i++) {
    center += points[i];
  }
  center = cv::Point3f(cv::Vec3f(center) / static_cast<int>(points.size()));

  for (size_t i = 0; i < points.size(); i++) {
    points[i] = points[i] - center;
  }
  center_point = center;
}
