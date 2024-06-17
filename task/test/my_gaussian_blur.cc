#include "gaussian_blur/gaussian_blur.h"
#include <glog/logging.h>
#include <memory>
#include <opencv2/opencv.hpp>

int main() {
  std::unique_ptr<GaussianBlur> gaussian_blur_ptr =
      std::make_unique<GaussianBlur>();

  double sigma = 1.0;
  double k_size = 3;
  std::vector<std::vector<double>> gaussian_kernel =
      gaussian_blur_ptr->builtGaussianKernel(3, 1.0);
  std::vector<std::vector<double>> mean_kernel =
      gaussian_blur_ptr->builtMeanKernel(3);

  cv::Mat img, gaussian_blur_result, mean_blur_result;
  img = cv::Mat::eye(5, 5, CV_8UC1);
  img *= 255;

  gaussian_blur_ptr->convolution(img, gaussian_blur_result, gaussian_kernel, 3);

  gaussian_blur_ptr->convolution(img, mean_blur_result, mean_kernel, 3);

  double sigma_x = sigma;
  double sigma_y = sigma_x;
  int k_size_w = k_size;
  int k_size_h = k_size_w;
  cv::Mat opencv_gaussian_blur_output;
  cv::GaussianBlur(img, opencv_gaussian_blur_output,
                   cv::Size(k_size_w, k_size_h), sigma_x, sigma_y,
                   cv::BORDER_CONSTANT);

  std::cout << "my gaussian blur:\n" << gaussian_blur_result << std::endl;
  std::cout << "my mean blur:\n" << mean_blur_result << std::endl;
  std::cout << "opencv gaussian blur:\n"
            << opencv_gaussian_blur_output << std::endl;
  return 0;
}
