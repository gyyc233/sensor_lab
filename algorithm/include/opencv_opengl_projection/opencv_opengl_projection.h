#include <Eigen/Dense>
#include <cmath>
#include <memory>
#include <opencv2/opencv.hpp>
#include <string>
#include <vector>

namespace Algorithm {

struct VirtualPinholeCameraParams {
  double c_x = 0;
  double c_y = 0;
  double f_x = 0;
  double f_y = 0;

  void update_fov(double fov_x_rad, double image_width, double fov_y_rad,
                  double image_height) {
    f_x = 0.5 * image_width / tan(fov_x_rad / 2);
    f_y = 0.5 * image_height / tan(fov_y_rad / 2);
  }
};

struct VirtualNDCParams {
  double width = 0;
  double height = 0;
  double near = 0;
  double far = 0;
};

/// @brief Converting OpenCV cameras to OpenGL cameras
class OpenCVToOpenGLProjection {
public:
  OpenCVToOpenGLProjection();
  virtual ~OpenCVToOpenGLProjection();

  void
  input_camera_intrinsics_mat(const VirtualPinholeCameraParams &camera_params);

  void input_ndc_params(const VirtualNDCParams &ndc_params);

  void input_camera_coordinates(
      const std::vector<Eigen::Vector3d> &camera_coordinates);

  bool process(std::vector<Eigen::Vector3d> &view_points);

private:
  Eigen::Matrix3d camera_matrix_; // camera intrinsics matrix
  VirtualPinholeCameraParams camera_params_;

  Eigen::Matrix4d rt_matrix_;

  Eigen::Matrix4d gl_projection_matrix_;
  VirtualNDCParams ndc_params_;

  std::vector<Eigen::Vector3d> camera_points_;
};

} // namespace Algorithm
