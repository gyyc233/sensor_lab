#include "dlt_camera_calibration/dlt_camera_calibration.h"
#include "spdlog/fmt/ostr.h"
#include "spdlog/fmt/ranges.h"
#include "spdlog/spdlog.h"

namespace Algorithm {

DltCameraCalibrationOperator::DltCameraCalibrationOperator() {
  world_points_.clear();
  image_points_.clear();
}

DltCameraCalibrationOperator::~DltCameraCalibrationOperator() {}

void DltCameraCalibrationOperator::setWorldPoints(
    const std::vector<cv::Point3d> &world_points) {
  world_points_ = world_points;
}

void DltCameraCalibrationOperator::setImagePoints(
    const std::vector<cv::Point2d> &image_points) {
  image_points_ = image_points;
}

int DltCameraCalibrationOperator::builtLinearEquations() {
  if (image_points_.empty() || image_points_.size() != world_points_.size()) {
    spdlog::warn("invalid size. image points size: {}, world points size: {}",
                 image_points_.size(), world_points_.size());
    return -2;
  }

  // built A matrix
  cv::Mat linear_m(image_points_.size() * 2, 12, CV_64FC1, cv::Scalar(0));

  for (size_t i = 0; i < image_points_.size(); i++) {
    double X = world_points_[i].x;
    double Y = world_points_[i].y;
    double Z = world_points_[i].z;
    double u = image_points_[i].x;
    double v = image_points_[i].y;

    double Xu = X * u;
    double Yu = Y * u;
    double Zu = Z * u;
    double Xv = X * v;
    double Yv = Y * v;
    double Zv = Z * v;

    linear_m.at<double>(i * 2, 0) = -X;
    linear_m.at<double>(i * 2, 1) = -Y;
    linear_m.at<double>(i * 2, 2) = -Z;
    linear_m.at<double>(i * 2, 3) = -1;

    linear_m.at<double>(i * 2, 8) = Xu;
    linear_m.at<double>(i * 2, 9) = Yu;
    linear_m.at<double>(i * 2, 10) = Zu;
    linear_m.at<double>(i * 2, 11) = u;

    linear_m.at<double>(i * 2 + 1, 4) = -X;
    linear_m.at<double>(i * 2 + 1, 5) = -Y;
    linear_m.at<double>(i * 2 + 1, 6) = -Z;
    linear_m.at<double>(i * 2 + 1, 7) = -1;

    linear_m.at<double>(i * 2 + 1, 8) = Xv;
    linear_m.at<double>(i * 2 + 1, 9) = Yv;
    linear_m.at<double>(i * 2 + 1, 10) = Zv;
    linear_m.at<double>(i * 2 + 1, 11) = v;
  }

  linear_equation_mat_ = std::move(linear_m);
  spdlog::debug("built linear equations");
  return 0;
}

int DltCameraCalibrationOperator::solve() {
  // A=U*S*V^t^
  cv::Mat U, S, Vt;
  cv::SVD::compute(linear_equation_mat_, S, U, Vt);

  cv::Mat l_vec(1, 12, CV_64FC1, cv::Scalar(0));
  // get last row
  l_vec = Vt(cv::Rect(0, 11, 12, 1));
  // make l_vec(0,11)==1.0
  l_vec = l_vec / l_vec.at<double>(0, 11);

  cv::Mat p_mat(3, 4, CV_64FC1, cv::Scalar(0));
  p_mat.at<double>(0, 0) = l_vec.at<double>(0, 0);
  p_mat.at<double>(0, 1) = l_vec.at<double>(0, 1);
  p_mat.at<double>(0, 2) = l_vec.at<double>(0, 2);
  p_mat.at<double>(0, 3) = l_vec.at<double>(0, 3);
  p_mat.at<double>(1, 0) = l_vec.at<double>(0, 4);
  p_mat.at<double>(1, 1) = l_vec.at<double>(0, 5);
  p_mat.at<double>(1, 2) = l_vec.at<double>(0, 6);
  p_mat.at<double>(1, 3) = l_vec.at<double>(0, 7);
  p_mat.at<double>(2, 0) = l_vec.at<double>(0, 8);
  p_mat.at<double>(2, 1) = l_vec.at<double>(0, 9);
  p_mat.at<double>(2, 2) = l_vec.at<double>(0, 10);
  p_mat.at<double>(2, 3) = l_vec.at<double>(0, 11);

  project_mat_ = std::move(p_mat);
  // spdlog::debug("project matrix: {}", project_mat_);
  return 0;
}

}; // namespace Algorithm
