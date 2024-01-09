#include "dlt_camera_calibration/dlt_camera_calibration.h"
#include "spdlog/spdlog.h"
#include <memory>

using namespace Algorithm;

int main() {
  spdlog::set_level(spdlog::level::debug);

  std::vector<cv::Point3d> points_3d;
  std::vector<cv::Point2d> points_2d = {
      {140, 285},   {908, 338},   {1655, 391},  {2450, 431},  {3223, 465},
      {4070, 535},  {4945, 552},  {200, 1045},  {941, 1112},  {1682, 1159},
      {2456, 1232}, {3243, 1265}, {4064, 1326}, {4878, 1392}, {661, 2100},
      {1435, 2166}, {2222, 2233}, {3043, 2306}, {3877, 2380}, {4744, 2460},
      {334, 2400},  {1148, 2460}, {1995, 2547}, {2849, 2627}, {3737, 2713},
      {4658, 2794}, {834, 2807},  {1722, 2907}, {2629, 2987}, {3570, 3080},
      {4551, 3167}, {474, 3214},  {1408, 3314}, {2389, 3401}, {3377, 3488},
      {4437, 3614}};

  // data
  std::vector<double> X_w = {216, 180, 144, 108, 72,  36,  0,   216, 180,
                             144, 108, 72,  36,  0,   180, 144, 108, 72,
                             36,  0,   180, 144, 108, 72,  36,  0,   144,
                             108, 72,  36,  0,   144, 108, 72,  36,  0};
  std::vector<double> Y_w = {72, 72, 72, 72, 72, 72, 72, 36, 36, 36, 36, 36,
                             36, 36, 0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
                             0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0};
  std::vector<double> Z_w = {0,   0,   0,   0,   0,   0,   0,   0,   0,
                             0,   0,   0,   0,   0,   36,  36,  36,  36,
                             36,  36,  72,  72,  72,  72,  72,  72,  108,
                             108, 108, 108, 108, 144, 144, 144, 144, 144};

  for (size_t i = 0; i < X_w.size(); i++) {
    cv::Point3d p(X_w[i], Y_w[i], Z_w[i]);
    points_3d.push_back(p);
  }

  std::shared_ptr<DltCameraCalibrationOperator> dlt_camera_calib_operator =
      std::make_shared<DltCameraCalibrationOperator>();
  dlt_camera_calib_operator->setImagePoints(points_2d);
  dlt_camera_calib_operator->setWorldPoints(points_3d);
  dlt_camera_calib_operator->builtLinearEquations();
  dlt_camera_calib_operator->solve();

  return 0;
}
