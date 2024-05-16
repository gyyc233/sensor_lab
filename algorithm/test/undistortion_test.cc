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
  // inicpp::IniManager ini_handle("./data/Config.ini");
  // int NvDistCenterLatOffsetMax =
  //     ini_handle["CheckTolerance"].toInt("NvDistCenterLatOffsetMax");
  // int NvDistCenterLatOffsetMin =
  //     ini_handle["CheckTolerance"].toInt("NvDistCenterLatOffsetMin");
  // std::cout << NvDistCenterLatOffsetMax << ", " << NvDistCenterLatOffsetMin
  //           << std::endl;

  // rapidcsv::Document doc("./data/ResDistCorrect.csv",
  //                        rapidcsv::LabelParams(-1, -1));

  // std::vector<std::vector<float>> points_xy;
  // for (int i = 3; i <= 11; i++) {
  //   std::vector<float> points = doc.GetRow<float>(i);
  //   points_xy.push_back(points);
  // }

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
