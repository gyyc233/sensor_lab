#include "monocular_camera_calib/monocular_camera_calib.h"
#include "stereo_calibration/stereo_calibration.h"
#include <glog/logging.h>
#include <memory>
#include <opencv2/opencv.hpp>

using namespace Algorithm;
using namespace std;

int main() {
  std::cout << "monocular calibration" << std::endl;
  // std::shared_ptr<MonocularCameraCalib> mono_camera_calib_operator =
  //     std::make_shared<MonocularCameraCalib>();
  // std::vector<std::string> files;
  // mono_camera_calib_operator->SingleCalibrate("./test_single_camera_calib.txt",
  //                                             "./data/camera_data/right");

  std::cout << "stereo calibration" << std::endl;
  std::string intrinsic_filename = "./data/stereo/intrinsics.yml";
  std::string extrinsic_filename = "./data/stereo/extrinsics.yml";
  std::string point_cloud_filename = "./data/stereo/point3D.txt";

  std::shared_ptr<StereoCalib> stereo_camera_calib_operator =
      std::make_shared<StereoCalib>();

  // stereo_camera_calib_operator->initFileList("./data/stereo", 1, 18);
  // stereo_camera_calib_operator->stereoCalibrate(intrinsic_filename,
  //                                               extrinsic_filename);

  stereo_camera_calib_operator->stereoMatch(
      "./data/stereo/stereo_test.jpg", intrinsic_filename, extrinsic_filename,
      false, point_cloud_filename);

  return 0;
}
