#include "opencv_opengl_projection/opencv_opengl_projection.h"

using namespace Algorithm;

int main() {
  std::cout << "========= projection test ==================" << std::endl;
  std::shared_ptr<OpenCVToOpenGLProjection> project_operator =
      std::make_shared<OpenCVToOpenGLProjection>();

  VirtualPinholeCameraParams camera_params;
  camera_params.c_x = 88;
  camera_params.c_y = 109;
  double fov = 45.0 / 360.0 * 2 * M_PI;
  camera_params.update_fov(fov, 178, fov, 218);
  project_operator->input_camera_intrinsics_mat(camera_params);

  VirtualNDCParams ndc_params;
  ndc_params.width = 178;
  ndc_params.height = 218;
  ndc_params.near = 10;
  ndc_params.far = 20;
  project_operator->input_ndc_params(ndc_params);

  std::vector<Eigen::Vector3d> camera_coordinates = {{1.0, 2.0, 15.0}};
  project_operator->input_camera_coordinates(camera_coordinates);

  std::vector<Eigen::Vector3d> view_points;
  project_operator->process(view_points);
  return 0;
}
