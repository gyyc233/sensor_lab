#include "orb_cv.h"
#include <iostream>
#include <memory>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>

int main() {
  std::unique_ptr<SensorLab::ORB_CV> orb_operator =
      std::make_unique<SensorLab::ORB_CV>();

  orb_operator->initialization();
  orb_operator->inputParams("./data/1.png", "./data/2.png");
  orb_operator->run();

  return 0;
}
