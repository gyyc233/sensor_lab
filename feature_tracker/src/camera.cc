#include "camera.h"
#include <opencv2/calib3d/calib3d.hpp>

namespace sensor_lab {

Camera::Parameters::Parameters(ModelType model_type)
    : model_type_(model_type), image_width_(0), image_height_(0) {
  switch (model_type) {
  case PINHOLE:
    number_intrinsics_ = 9;
    break;

  default:
    number_intrinsics_ = 9;
  }
}

Camera::Parameters::Parameters(ModelType model_type,
                               const std::string &camera_name, int w, int h)
    : model_type_(model_type), camera_name_(camera_name), image_width_(w),
      image_height_(h) {
  switch (model_type) {
  case PINHOLE:
    number_intrinsics_ = 9;
    break;

  default:
    number_intrinsics_ = 9;
  }
}

Camera::ModelType &Camera::Parameters::modelType(void) { return model_type_; }

std::string &Camera::Parameters::cameraName(void) { return camera_name_; }

int &Camera::Parameters::imageWidth(void) { return image_width_; }

int &Camera::Parameters::imageHeight(void) { return image_height_; }

Camera::ModelType Camera::Parameters::modelType(void) const {
  return model_type_;
}

const std::string &Camera::Parameters::cameraName(void) const {
  return camera_name_;
}

int Camera::Parameters::imageWidth(void) const { return image_width_; }

int Camera::Parameters::imageHeight(void) const { return image_height_; }

int Camera::Parameters::nIntrinsics(void) const { return number_intrinsics_; }

// Camera::Parameters

cv::Mat &Camera::mask(void) { return mask_; }

const cv::Mat &Camera::mask(void) const { return mask_; }

void Camera::estimateExtrinsics(const std::vector<cv::Point3f> &object_points,
                                const std::vector<cv::Point2f> &image_points,
                                cv::Mat &r_vec, cv::Mat &t_vec) const {
  std::vector<cv::Point2f> Ms(image_points.size());
  for (size_t i = 0; i < Ms.size(); ++i) {
    // 投影到归一化平面
    Eigen::Vector3d P;
    liftProjective(Eigen::Vector2d(imagePoints.at(i).x, imagePoints.at(i).y),
                   P);

    P /= P(2);

    Ms.at(i).x = P(0);
    Ms.at(i).y = P(1);
  }

  // assume unit focal length, zero principal point, and zero distortion
  // 假设单位焦距、零主点和零失真,这里相机内参用的单位阵
  cv::solvePnP(object_points, Ms, cv::Mat::eye(3, 3, CV_64F), cv::noArray(),
               r_vec, t_vec);
}

double Camera::reprojectionDist(const Eigen::Vector3d &P1,
                                const Eigen::Vector3d &P2) const {
  Eigen::Vector2d p1, p2;

  spaceToPlane(P1, p1);
  spaceToPlane(P2, p2);

  return (p1 - p2).norm();
}

double Camera::reprojectionError(
    const std::vector<std::vector<cv::Point3f>> &object_points,
    const std::vector<std::vector<cv::Point2f>> &image_points,
    const std::vector<cv::Mat> &r_vecs, const std::vector<cv::Mat> &t_vecs,
    cv::OutputArray _per_view_errors) const {
  int image_count = object_points.size();
  size_t points_so_far = 0;
  double total_error = 0.0;

  bool computer_per_view_errors = _per_view_errors.needed();
  cv::Mat per_view_errors;
  if (computer_per_view_errors) {
    _per_view_errors.create(image_count, 1, CV_64F);
    per_view_errors = _per_view_errors.getMat();
  }

  for (int i = 0; i < image_count; ++i) {
    size_t point_count = image_count.at(i).size();
    points_so_far += point_count;

    // world frame points to image points [u,v]
    std::vector<cv::Point2f> est_image_points;
    projectPoints(object_points.at(i), r_vecs.at(i), t_vecs.at(i),
                  est_image_points);

    double err = 0.0;
    for (size_t j = 0; j < image_points.at(i).size(); ++j) {
      err += cv::norm(image_points.at(i).at(j) - est_image_points.at(j));
    }

    if (computer_per_view_errors) {
      per_view_errors.at<double>(i) = err / point_count;
    }

    total_error += err;
  }

  return total_error / points_so_far;
}

double Camera::reprojectionError(const Eigen::Vector3d &P,
                                 const Eigen::Quaterniond &camera_q,
                                 const Eigen::Vector3d &camera_t,
                                 const Eigen::Vector2d &observed_p) const {
  // world coordinate to camera coordinate
  Eigen::Vector3d P_cam = camera_q.toRotationMatrix() * P + camera_t;

  // camera coordinate to image
  Eigen::Vector2d p;
  spaceToPlane(P_cam, p);

  return (p - observed_p).norm();
}

void Camera::projectPoints(const std::vector<cv::Point3f> &object_points,
                           const cv::Mat &r_vec, const cv::Mat &t_vec,
                           std::vector<cv::Point2f> &image_points) const {
  // project 3D object points to the image plane
  image_points.reserve(object_points.size());

  // 旋转向量转旋转矩阵
  cv::Mat R0;
  cv::Rodrigues(r_vec, R0);

  Eigen::MatrixXd R(3, 3);
  R << R0.at<double>(0, 0), R0.at<double>(0, 1), R0.at<double>(0, 2),
      R0.at<double>(1, 0), R0.at<double>(1, 1), R0.at<double>(1, 2),
      R0.at<double>(2, 0), R0.at<double>(2, 1), R0.at<double>(2, 2);

  Eigen::Vector3d t;
  t << t_vec.at<double>(0), t_vec.at<double>(1), t_vec.at<double>(2);

  for (size_t i = 0; i < object_points.size(); ++i) {
    const cv::Point3f &object_point = object_points.at(i);

    // Rotate and translate
    Eigen::Vector3d P;
    P << object_point.x, object_point.y, object_point.z;

    P = R * P + t;

    // to image frame [u,v]
    Eigen::Vector2d p;
    spaceToPlane(P, p);

    imagePoints.push_back(cv::Point2f(p(0), p(1)));
  }
}

} // namespace sensor_lab
