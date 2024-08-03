#include "epipolar_constraint.h"
#include "triansgulation.h"
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

  // get epipolar result
  std::vector<cv::DMatch> orb_match_result;
  std::vector<cv::KeyPoint> key_points_img_l;
  std::vector<cv::KeyPoint> key_points_img_r;
  epipolar_ptr->getMatchResult(key_points_img_l, key_points_img_r,
                               orb_match_result);

  cv::Mat rot_mat, translate_mat;
  epipolar_ptr->getTransformation(rot_mat, translate_mat);
  std::vector<cv::Mat> data;
  epipolar_ptr->gateData(data);

  std::unique_ptr<Triansgulation> triansgulation_ptr(new Triansgulation());
  triansgulation_ptr->inputCameraIntrinsics(camera_intrinsics);
  std::vector<cv::Point3d> points;
  triansgulation_ptr->triangulation(key_points_img_l, key_points_img_r,
                                    orb_match_result, rot_mat, translate_mat,
                                    points);

  return 0;
}
