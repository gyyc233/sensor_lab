#include "undistortion/undistortion.h"
#include <glog/logging.h>

namespace Algorithm {
Undistortion::Undistortion() {}

Undistortion::~Undistortion() {}

void Undistortion::inputParams(const std::string &image_path,
                               const std::vector<double> &intrinsics,
                               const std::vector<double> &distortion_params) {
  input_image_ = cv::imread(image_path, cv::IMREAD_GRAYSCALE);
  assert(input_image_.rows != 0);
  undistortion_image_ = cv::Mat(input_image_.rows, input_image_.cols, CV_8UC1);

  assert(intrinsics.size() == 4);
  fx_ = intrinsics[0];
  fy_ = intrinsics[1];
  cx_ = intrinsics[2];
  cy_ = intrinsics[3];

  assert(distortion_params.size() == 5);
  k1_ = distortion_params[0];
  k2_ = distortion_params[1];
  k3_ = distortion_params[2];
  p1_ = distortion_params[3];
  p2_ = distortion_params[4];
}

void Undistortion::inputParams(const cv::Mat &image,
                               const std::vector<double> &intrinsics,
                               const std::vector<double> &distortion_params) {
  input_image_ = image;
  assert(input_image_.rows != 0);
  undistortion_image_ = cv::Mat(input_image_.rows, input_image_.cols, CV_8UC1);

  assert(intrinsics.size() == 4);
  fx_ = intrinsics[0];
  fy_ = intrinsics[1];
  cx_ = intrinsics[2];
  cy_ = intrinsics[3];

  assert(distortion_params.size() == 5);
  k1_ = distortion_params[0];
  k2_ = distortion_params[1];
  k3_ = distortion_params[2];
  p1_ = distortion_params[3];
  p2_ = distortion_params[4];
}

bool Undistortion::run() {
  for (size_t v = 0; v < undistortion_image_.rows; v++) {
    for (size_t u = 0; u < undistortion_image_.cols; u++) {
      // pixel data to image data
      double image_x = (u - cx_) / fx_;
      double image_y = (v - cy_) / fy_;

      // undistorted for image data
      double r = sqrt(image_x * image_x + image_y * image_y);
      double undistorted_image_x =
          image_x * (1 + k1_ * r * r + k2_ * r * r * r * r +
                     k3_ * r * r * r * r * r * r) +
          2 * p1_ * image_x * image_y + p2_ * (r * r + 2 * image_x * image_x);
      double undistorted_image_y =
          image_y * (1 + k1_ * r * r + k2_ * r * r * r * r +
                     k3_ * r * r * r * r * r * r) +
          2 * p1_ * (r * r + 2 * image_x * image_x) + p2_ * image_x * image_y;

      // image data to pixel data
      double undistorted_u = fx_ * undistorted_image_x + cx_;
      double undistorted_v = fy_ * undistorted_image_y + cy_;

      // remain valid data
      if (undistorted_u >= 0 && undistorted_v >= 0 &&
          undistorted_u < undistortion_image_.cols &&
          undistorted_v < undistortion_image_.rows) {
        undistortion_image_.at<uchar>(v, u) =
            input_image_.at<uchar>((int)undistorted_v, (int)undistorted_u);
      } else {
        undistortion_image_.at<uchar>(v, u) = 0;
        std::cout << "invalid undistorted u: " << undistorted_u
                  << ", undistorted v: " << undistorted_v << std::endl;
      }
    }
  }
}

void Undistortion::output(cv::Mat &undistorted_image,
                          cv::Mat &distorted_image) {
  undistorted_image = undistortion_image_;
  distorted_image = input_image_;
}

cv::Mat Undistortion::getOutput() { return undistortion_image_; }

}; // namespace Algorithm
