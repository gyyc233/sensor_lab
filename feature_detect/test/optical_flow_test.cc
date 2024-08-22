#include "LK_optical_flow.h"
#include <iostream>
#include <memory>

using namespace SensorLab;

int main(int argc, char **argv) {
  std::unique_ptr<LKOpticalFlow> lkof_operator =
      std::make_unique<LKOpticalFlow>();

  lkof_operator->inputParams("./data/optical_flow/car1.jpg",
                             "./data/optical_flow/car2.jpg");

  cv::Mat pre_img =
      cv::imread("./data/optical_flow/car1.jpg", cv::IMREAD_UNCHANGED);

  lkof_operator->draw_optical_flow(pre_img);

  lkof_operator->cv_single_of_api("./data/optical_flow/car1.jpg",
                                  "./data/optical_flow/car2.jpg");

  return 0;
}
