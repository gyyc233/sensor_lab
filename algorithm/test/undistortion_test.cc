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
  std::string origin_image_path = "./data/origin.jpg";
  std::string demo_image_path = "./data/demo.jpg";
  std::shared_ptr<Undistortion> undistortion_ptr =
      std::make_shared<Undistortion>();
  std::vector<int> points_origin_x, points_origin_y;
  std::vector<int> points_demo_x, points_demo_y;
  undistortion_ptr->detectConners(origin_image_path, points_origin_x,
                                  points_origin_y);
  undistortion_ptr->detectConners(demo_image_path, points_demo_x,
                                  points_demo_y);

  undistortion_ptr->normalization(points_demo_x, points_demo_y);
}
