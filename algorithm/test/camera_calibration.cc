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
  std::string intrinsic_filename = "./data/camera_data_mix/intrinsics.yml";
  std::string extrinsic_filename = "./data/camera_data_mix/extrinsics.yml";
  std::string point_cloud_filename = "./data/camera_data_mix/point3D.txt";

  std::shared_ptr<StereoCalib> stereo_camera_calib_operator =
      std::make_shared<StereoCalib>();

  // slambook14 stereo demo
  // stereo_camera_calib_operator->stereoDemo("./data/stereo/left.png",
  //                                       "./data/stereo/right.png");

  // stereo_camera_calib_operator->initFileList("./data/camera_data_mix/mix", 1,
  //                                            8);
  // stereo_camera_calib_operator->stereoCalibrate(intrinsic_filename,
  //                                               extrinsic_filename);

  // stereo_camera_calib_operator->initFileList("./data/camera_data_mix/mix", 1,
  //                                            2);
  // stereo_camera_calib_operator->stereoMatch(
  //     0, intrinsic_filename, extrinsic_filename, false,
  //     point_cloud_filename);

  // 相应原始图像水平拼接
  // cv::Mat image_left;
  // cv::Mat image_right;
  // cv::Mat output;

  // image_left= cv::imread("./data/camera_data_mix/left/left01.jpg");
  // image_right= cv::imread("./data/camera_data_mix/right/right01.jpg");

  // cv::hconcat(image_left, image_right, output);
  // cv::imwrite("./data/camera_data_mix/mix/01.jpg",output);
  return 0;
}
