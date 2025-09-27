#include "monocular_camera_calib/monocular_camera_calib.h"
#include "stereo_calibration/stereo_calibration.h"
#include <glog/logging.h>
#include <memory>

using namespace Algorithm;
using namespace std;

int main() {
  std::cout << "monocular calibration" << std::endl;
  std::shared_ptr<MonocularCameraCalib> mono_camera_calib_operator =
      std::make_shared<MonocularCameraCalib>();
  std::vector<std::string> files;
  mono_camera_calib_operator->SingleCalibrate("./test_single_camera_calib.txt",
                                              "./data/camera_data/left");

  std::cout << "stereo calibration" << std::endl;
  return 0;
}
