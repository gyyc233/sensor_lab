#include "epipolar_constraint.h"
#include <iostream>
#include <memory>
#include <vector>

using namespace SensorLab;

int main(int argc, char **argv) {
  std::unique_ptr<EpipolarConstraint> epipolar_ptr =
      std::make_unique<EpipolarConstraint>();
  epipolar_ptr->inputParams("./data/1.png", "./data/2.png");
  std::vector<double> camera_intrinsics = {520.9, 521.0, 325.1, 249.7};
  double focal = 520.0;
  epipolar_ptr->inputCameraIntrinsics(camera_intrinsics, focal);
  epipolar_ptr->inputPrincipalPoints(325.1, 249.7);
  epipolar_ptr->run();

  std::vector<double> quaternion, translate;
  epipolar_ptr->output(quaternion, translate);
  return 0;
}
