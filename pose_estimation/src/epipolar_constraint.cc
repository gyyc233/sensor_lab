#include "epipolar_constraint.h"
#include "transform/cv_eigen_convert.hpp"
#include "transform/eigenGeometryTransfer.hpp"

using namespace SensorLab;

EpipolarConstraint::EpipolarConstraint() {
  std::cout << "construct EpipolarConstraint" << std::endl;
  feature_detect_ptr_ = std::make_unique<ORB_CV>();
}

EpipolarConstraint::~EpipolarConstraint() {
  std::cout << "destruct EpipolarConstraint" << std::endl;
}

void EpipolarConstraint::inputParams(const char *left_img,
                                     const char *right_img) {
  feature_detect_ptr_->initialization();
  feature_detect_ptr_->inputParams(left_img, right_img);
  feature_detect_ptr_->run();

  orb_match_result_.clear();
  key_points_img_l_.clear();
  key_points_img_r_.clear();
  feature_detect_ptr_->output(orb_match_result_, key_points_img_l_,
                              key_points_img_r_);
}

void EpipolarConstraint::initialization() { std::cout << "todo" << std::endl; }

void EpipolarConstraint::inputCameraIntrinsics(
    const std::vector<double> &intrinsics, double focal) {
  assert(intrinsics.size() == 4);
  camera_intrinsics_mat_ =
      (cv::Mat_<double>(3, 3) << intrinsics[0], 0, intrinsics[2], 0,
       intrinsics[1], intrinsics[3], 0, 0, 1);
  focal_ = focal;
}

void EpipolarConstraint::inputPrincipalPoints(double u, double v) {
  principal_point_.x = u;
  principal_point_.y = v;
}

void EpipolarConstraint::pose_estimation_2d2d(
    std::vector<cv::KeyPoint> &key_points_l,
    std::vector<cv::KeyPoint> &key_points_r, std::vector<cv::DMatch> &matches,
    cv::Mat &R, cv::Mat &t) {
  std::vector<cv::Point2f> points_l;
  std::vector<cv::Point2f> points_r;
  for (size_t i = 0; i < matches.size(); i++) {
    points_l.push_back(
        key_points_l[matches[i].queryIdx].pt); // queryIdx -- first input image
    points_r.push_back(
        key_points_r[matches[i].trainIdx].pt); // trainIdx -- second input image
  }

  // 1. 计算基础矩阵
  cv::Mat fundamental_matrix;
  fundamental_matrix =
      cv::findFundamentalMat(points_l, points_r, cv::FM_8POINT);
  std::cout << "fundamental_matrix is: \n" << fundamental_matrix << std::endl;

  // 2. 计算本质矩阵
  cv::Mat essential_matrix;
  essential_matrix =
      cv::findEssentialMat(points_l, points_r, focal_, principal_point_);
  std::cout << "essential_matrix is \n" << essential_matrix << std::endl;

  // 3. 计算单应矩阵
  //-- 但是本例中场景不是平面，单应矩阵意义不大
  cv::Mat homography_matrix;
  homography_matrix = cv::findHomography(points_l, points_r, cv::RANSAC, 3);
  std::cout << "homography_matrix is \n" << homography_matrix << std::endl;

  // 4. 从本质矩阵中恢复旋转和平移信息
  cv::recoverPose(essential_matrix, points_l, points_r, rotation_, translate_,
                  focal_, principal_point_);

  std::cout << "rotation is \n" << rotation_ << std::endl;
  std::cout << "translate is \n " << translate_ << std::endl;
}

void EpipolarConstraint::run() {
  pose_estimation_2d2d(key_points_img_l_, key_points_img_r_, orb_match_result_,
                       rotation_, translate_);
}

void EpipolarConstraint::output(std::vector<double> &quaternion,
                                std::vector<double> &translation) {
  auto rotation_mat_xd = cv_mat_convert_to_eigen<double>(rotation_);
  Eigen::Matrix3d rotation_mat;
  rotation_mat << rotation_mat_xd(0, 0), rotation_mat_xd(0, 1),
      rotation_mat_xd(0, 2), rotation_mat_xd(1, 0), rotation_mat_xd(1, 1),
      rotation_mat_xd(1, 2), rotation_mat_xd(2, 0), rotation_mat_xd(2, 1),
      rotation_mat_xd(2, 2);

  auto quat = rotationMatrix2Quaternion(rotation_mat);
  quaternion = {quat.w(), quat.x(), quat.y(), quat.z()};

  auto translate_vec = cv_mat_convert_to_vector_2d<double>(translate_);
  translation = {translate_vec[0][0], translate_vec[1][0], translate_vec[2][0]};
}

void EpipolarConstraint::getMatchResult(std::vector<cv::KeyPoint> &points_l,
                                        std::vector<cv::KeyPoint> &points_r,
                                        std::vector<cv::DMatch> &match) {
  points_l = key_points_img_l_;
  points_r = key_points_img_r_;
  match = orb_match_result_;
}

void EpipolarConstraint::getTransformation(cv::Mat &rot, cv::Mat &translation) {
  rot = rotation_;
  translation = translate_;
}

void EpipolarConstraint::gateData(std::vector<cv::Mat> &data) {
  feature_detect_ptr_->getData(data);
}
