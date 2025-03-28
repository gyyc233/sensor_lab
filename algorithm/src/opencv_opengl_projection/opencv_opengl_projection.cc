#include "opencv_opengl_projection/opencv_opengl_projection.h"

using namespace Algorithm;

OpenCVToOpenGLProjection::OpenCVToOpenGLProjection() {
  camera_matrix_ = Eigen::Matrix3d::Identity();
  rt_matrix_ = Eigen::Matrix4d::Identity();
  gl_projection_matrix_ = Eigen::Matrix4d::Zero();
  gl_projection_matrix_(3, 2) = -1;
}

OpenCVToOpenGLProjection::~OpenCVToOpenGLProjection() {}

void OpenCVToOpenGLProjection::input_camera_intrinsics_mat(
    const VirtualPinholeCameraParams &camera_params) {
  camera_params_ = camera_params;

  camera_params_.f_x = camera_params_.f_y;

  camera_matrix_(0, 0) = camera_params_.f_x;
  camera_matrix_(1, 1) = camera_params_.f_y;
  camera_matrix_(0, 2) = camera_params_.c_x;
  camera_matrix_(1, 2) = camera_params_.c_y;

  std::cout << "camera mat:\n" << camera_matrix_ << std::endl;
}

void OpenCVToOpenGLProjection::input_ndc_params(
    const VirtualNDCParams &ndc_params) {
  ndc_params_ = ndc_params;

  gl_projection_matrix_(0, 0) = 2 * camera_params_.f_x / ndc_params_.width;
  gl_projection_matrix_(0, 2) =
      (ndc_params_.width - 2 * camera_params_.c_x) / ndc_params_.width;

  gl_projection_matrix_(1, 1) = -2 * camera_params_.f_y / ndc_params_.height;
  gl_projection_matrix_(1, 2) =
      (2 * camera_params_.c_y / ndc_params_.height) - 1;

  gl_projection_matrix_(2, 2) = (-ndc_params_.near - ndc_params_.far) /
                                (ndc_params_.far - ndc_params_.near);
  gl_projection_matrix_(2, 3) = -2 * ndc_params_.far * ndc_params_.near /
                                (ndc_params_.far - ndc_params_.near);

  std::cout << "project mat:\n" << gl_projection_matrix_ << std::endl;
}

void OpenCVToOpenGLProjection::input_camera_coordinates(
    const std::vector<Eigen::Vector3d> &camera_coordinates) {
  camera_points_ = camera_coordinates;
}

bool OpenCVToOpenGLProjection::process(
    std::vector<Eigen::Vector3d> &view_points) {
  view_points.clear();

  for (int i = 0; i < camera_points_.size(); i++) {
    // flip the point z coord
    camera_points_[i](2, 0) *= -1;
    std::cout << "camera_points_: " << camera_points_[i] << std::endl;

    Eigen::Vector4d camera_point = {camera_points_[i](0, 0),
                                    camera_points_[i](1, 0),
                                    camera_points_[i](2, 0), 1};
    Eigen::Vector4d ndc_point = gl_projection_matrix_ * camera_point;
    ndc_point /= ndc_point(3, 0);
    ndc_point(0, 0) = (ndc_point(0, 0) + 1.0) * ndc_params_.width;
    ndc_point(1, 0) = (ndc_point(1, 0) + 1.0) * ndc_params_.height;
    ndc_point(2, 0) = ndc_point(2, 0) + 1.0;
    ndc_point(3, 0) = ndc_point(3, 0) + 1.0;
    ndc_point /= 2.0;
    std::cout << "ndc point: " << ndc_point << std::endl;

    // ndc to screen
    // 这里好像有问题 TODO:
    Eigen::Vector3d screen_point = {
        (ndc_point(0, 0) + 1.0) * 0.5 * ndc_params_.width,
        (1.0 - ndc_point(1, 0)) * 0.5 * ndc_params_.height, 1};
    std::cout << "screen point: " << screen_point << std::endl;
    view_points.emplace_back(screen_point);
  }

  return true;
}
