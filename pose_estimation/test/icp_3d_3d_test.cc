#include "icp_3d3d.h"
#include "icp_g2o.h"
#include <iostream>
#include <memory>
#include <vector>

using namespace SensorLab;

int main(int argc, char **argv) {
  std::unique_ptr<ICP_3D3D> icp_3d3d_ptr = std::make_unique<ICP_3D3D>();

  icp_3d3d_ptr->inputParams("./data/1.png", "./data/2.png",
                            "./data/1_depth.png", "./data/2_depth.png");

  std::vector<double> camera_intrinsics = {520.9, 521.0, 325.1, 249.7};
  icp_3d3d_ptr->inputCameraIntrinsics(camera_intrinsics, 5000.0);

  icp_3d3d_ptr->run();

  // TODO: failed to ICP on g2o
  // std::vector<cv::Point3f> source_points;
  // std::vector<cv::Point3f> target_points;
  // icp_3d3d_ptr->getPointsData(source_points, target_points);

  // std::unique_ptr<ICP_G2O> icp_g2o_ptr = std::make_unique<ICP_G2O>();
  // cv::Mat rotation_matrix;
  // cv::Mat translation;
  // icp_g2o_ptr->bundleAdjustment(source_points, target_points,
  // rotation_matrix,
  //                               translation);
  return 0;
};
