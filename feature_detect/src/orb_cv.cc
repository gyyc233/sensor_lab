#include "orb_cv.h"

using namespace SensorLab;

ORB_CV::ORB_CV() {}

ORB_CV::~ORB_CV() {}

void ORB_CV::initialization() {
  detector_ = cv::ORB::create();
  descriptor_ = cv::ORB::create();
  matcher_ = cv::DescriptorMatcher::create("BruteForce-Hamming");
}

void ORB_CV::run() {
  std::vector<cv::KeyPoint> key_points_img_l, key_points_img_r;
  detector_->detect(img_l_, key_points_img_l);
  detector_->detect(img_r_, key_points_img_r);
}

void ORB_CV::inputParams(const char *left_img, const char *right_img) {
  img_l_ = cv::imread(left_img, cv::IMREAD_COLOR);
  img_r_ = cv::imread(right_img, cv::IMREAD_COLOR);

  assert(img_l_.data != nullptr && img_r_.data != nullptr);
}
