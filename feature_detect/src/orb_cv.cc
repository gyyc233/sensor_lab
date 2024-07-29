#include "orb_cv.h"
#include <opencv2/imgcodecs.hpp>

using namespace SensorLab;

ORB_CV::ORB_CV() {}

ORB_CV::~ORB_CV() {}

void ORB_CV::initialization() {
  detector_ = cv::ORB::create();
  orb_descriptor_ = cv::ORB::create();
  matcher_ = cv::DescriptorMatcher::create("BruteForce-Hamming");
}

void ORB_CV::run() {
  // 1. detect oriented FAST corners
  std::vector<cv::KeyPoint> key_points_img_l, key_points_img_r;
  detector_->detect(img_l_, key_points_img_l);
  detector_->detect(img_r_, key_points_img_r);

  // 2. based on FAST corners compute BRIEF descriptors
  cv::Mat descriptors_l, descriptors_r;
  detector_->compute(img_l_, key_points_img_l, descriptors_l);
  detector_->compute(img_r_, key_points_img_r, descriptors_r);

  // draw FAST corners
  cv::Mat fast_corners_l_output, fast_corners_r_output;
  cv::drawKeypoints(img_l_, key_points_img_l, fast_corners_l_output,
                    cv::Scalar::all(-1));
  cv::drawKeypoints(img_r_, key_points_img_r, fast_corners_r_output,
                    cv::Scalar::all(-1));

  cv::imwrite("fast_corners_l_output.png", fast_corners_l_output);
  cv::imwrite("fast_corners_r_output.png", fast_corners_r_output);

  // 3. match for BRIEF descriptors, using Hamming distance
  std::vector<cv::DMatch> matches;
  matcher_->match(descriptors_l, descriptors_r, matches);

  // 4. select matches points part
  // 找出所有匹配之间的最小距离和最大距离,
  // 即是最相似的和最不相似的两组点之间的距离
  double min_dist = 10000, max_dist = 0;
  for (int i = 0; i < descriptors_l.rows; i++) {
    double dist = matches[i].distance;
    if (dist < min_dist)
      min_dist = dist; //最短距离，最相似
    if (dist > max_dist)
      max_dist = dist; //最长距离，最不相似
  }

  std::cout << "min_dist: " << min_dist << std::endl;
  std::cout << "max_dist: " << max_dist << std::endl;

  good_matches_.clear();
  for (int i = 0; i < descriptors_l.rows; i++) {
    // 当描述子之间的距离大于两倍的最小距离时,即认为匹配有误.但有时候最小距离会非常小,设置一个经验值30作为下限
    if (matches[i].distance <= std::max(2 * min_dist, 30.0)) {
      good_matches_.push_back(matches[i]);
    }
  }

  // save orb detect result
  cv::Mat image_origin_match;
  cv::Mat image_good_match;
  cv::drawMatches(img_l_, key_points_img_l, img_r_, key_points_img_r, matches,
                  image_origin_match);
  cv::drawMatches(img_l_, key_points_img_l, img_r_, key_points_img_r,
                  good_matches_, image_good_match);
  cv::imwrite("image_origin_match.png", image_origin_match);
  cv::imwrite("image_good_match.png", image_good_match);
}

void ORB_CV::inputParams(const char *left_img, const char *right_img) {
  img_l_ = cv::imread(left_img, cv::IMREAD_COLOR);
  img_r_ = cv::imread(right_img, cv::IMREAD_COLOR);

  assert(img_l_.data != nullptr && img_r_.data != nullptr);
}
