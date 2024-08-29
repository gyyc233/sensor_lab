#include "LK_optical_flow.h"
#include "ctf_LK_optical_flow.h"
#include <iostream>
#include <memory>

using namespace SensorLab;

int main(int argc, char **argv) {
  std::cout << "1. single LK" << std::endl;
  std::unique_ptr<LKOpticalFlow> lk_of_operator =
      std::make_unique<LKOpticalFlow>();

  lk_of_operator->inputParams("./data/optical_flow/car1.jpg",
                              "./data/optical_flow/car2.jpg");

  cv::Mat pre_img =
      cv::imread("./data/optical_flow/car1.jpg", cv::IMREAD_UNCHANGED);

  lk_of_operator->draw_optical_flow(pre_img);

  lk_of_operator->cv_single_of_api("./data/optical_flow/car1.jpg",
                                   "./data/optical_flow/car2.jpg");

  std::cout << "\n\n2. ctf LK" << std::endl;
  std::unique_ptr<CtfLKOpticalFlow> ctf_lk_of_operator =
      std::make_unique<CtfLKOpticalFlow>();

  ctf_lk_of_operator->inputParams("./data/optical_flow/car1.jpg",
                                  "./data/optical_flow/car2.jpg");
  ctf_lk_of_operator->draw_optical_flow(pre_img);

  return 0;
}
