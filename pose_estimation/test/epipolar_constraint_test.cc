#include "epipolar_constraint.h"
#include <iostream>
#include <memory>

using namespace SensorLab;

int main(int argc, char **argv) {
  std::unique_ptr<EpipolarConstraint> my_ptr =
      std::make_unique<EpipolarConstraint>();
  my_ptr->inputParams("./data/1.png", "./data/2.png");
  return 0;
}
