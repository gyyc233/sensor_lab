#include "bundle_adjustment_gauss_newton.h"
#include "triansgulation.h"
#include <iostream>
#include <memory>
#include <vector>

using namespace SensorLab;

int main(int argc, char **argv) {
  std::unique_ptr<BA_GaussNewton> ba_gauss_newton =
      std::make_unique<BA_GaussNewton>();

  ba_gauss_newton->inputParams("./data/1.png", "./data/2.png",
                               "./data/1_depth.png", "./data/2_depth.png");

  std::vector<double> camera_intrinsics = {520.9, 521.0, 325.1, 249.7};
  ba_gauss_newton->inputCameraIntrinsics(camera_intrinsics);
  ba_gauss_newton->setIteration(50);
  ba_gauss_newton->run();
  return 0;
}
