#include "bundle_adjustment_g2o.h"
#include <iostream>
#include <memory>
#include <vector>
using namespace SensorLab;

int main() {
  std::unique_ptr<BA_G2o> ba_g2o_GN_ptr = std::make_unique<BA_G2o>();

  ba_g2o_GN_ptr->inputParams("./data/1.png", "./data/2.png",
                             "./data/1_depth.png", "./data/2_depth.png");

  std::vector<double> camera_intrinsics = {520.9, 521.0, 325.1, 249.7};
  ba_g2o_GN_ptr->inputCameraIntrinsics(camera_intrinsics);
  ba_g2o_GN_ptr->setIteration(50);
  ba_g2o_GN_ptr->run();
  ba_g2o_GN_ptr->runG2o();
}
