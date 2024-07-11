// #include "common/inicpp.hpp"
// #include "common/rapidcsv.h"
#include "undistortion/undistortion.h"
#include <fstream>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

using namespace Algorithm;
int main() {
  std::unique_ptr<Undistortion> undistortion_ptr =
      std::make_unique<Undistortion>();
  std::string image_path = "./data/distorted.png";
  std::vector<double> camera_intrinsics = {458.654, 457.296, 367.215, 248.375};
  std::vector<double> distortion_params = {-0.28340811, 0.07395907, 0,
                                           0.00019359, 1.76187114e-05};
  undistortion_ptr->inputParams(image_path, camera_intrinsics,
                                distortion_params);
  undistortion_ptr->run();

  cv::Mat input_image;
  cv::Mat output_image;
  undistortion_ptr->output(output_image, input_image);
  cv::imshow("distorted", input_image);
  cv::imshow("undistorted", output_image);
  cv::waitKey();
  return 0;
}
