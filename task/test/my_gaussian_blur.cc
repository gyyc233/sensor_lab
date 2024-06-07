#include "gaussian_blur/gaussian_blur.h"
#include <glog/logging.h>
#include <memory>
#include <opencv2/opencv.hpp>

int main() {
  std::unique_ptr<GaussianBlur> gaussian_blur_ptr =
      std::make_unique<GaussianBlur>();
  std::vector<std::vector<double>> gaussian_kernel =
      gaussian_blur_ptr->builtGaussianKernel(3, 0.5);
  std::vector<std::vector<double>> mean_kernel =
      gaussian_blur_ptr->builtMeanKernel(3);

  cv::Mat img, gaussian_blur_result, mean_blur_result;
  img = cv::Mat::eye(5, 5, CV_8UC1);
  img *= 255;

  // gaussian_blur_ptr->convolution(img, gaussian_blur_result, gaussian_kernel,
  // 3);
  gaussian_blur_ptr->convolution(img, mean_blur_result, mean_kernel, 3);

  std::cout << "my gaussian blur:\n" << gaussian_blur_result << std::endl;
  std::cout << "my mean blur:\n" << mean_blur_result << std::endl;
  return 0;
}
