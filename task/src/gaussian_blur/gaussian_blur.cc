#include "gaussian_blur/gaussian_blur.h"

GaussianBlur::GaussianBlur() {}

GaussianBlur::~GaussianBlur() {}

void GaussianBlur::gaussianBlur(const cv::Mat &srcImage, cv::Mat &dstImage,
                                int ksize, double sigma) const {}

void GaussianBlur::convolution(const cv::Mat &myImage, cv::Mat &Result,
                               const std::vector<std::vector<double>> &kernel,
                               int ksize) const {}

std::vector<std::vector<double>>
GaussianBlur::builtGaussianKernel(int ksize, double sigma) const {
  double sum = 0.0;
  int center = ksize / 2;

  // create ksize*ksize
  std::vector<std::vector<double>> arr(ksize, std::vector<double>(ksize, 0));

  for (int i = 0; i < arr.size(); i++) {
    for (int j = 0; j < arr[0].size(); j++) {
      arr[i][j] =
          exp(-((i - center) * (i - center) + (j - center) * (j - center)) /
              (sigma * sigma * 2));
      sum += arr[i][j];
    }
  }

  double check_sum = 0;
  for (int i = 0; i < arr.size(); i++) {
    for (int j = 0; j < arr[0].size(); j++) {
      arr[i][j] /= sum;
      check_sum += arr[i][j];
    }
  }

  printf("check sum: %.10f \n", check_sum);
  printf("printf gaussian kernel, size: %d \n", ksize);
  for (int i = 0; i < arr.size(); i++) {
    for (int j = 0; j < arr[0].size(); j++) {
      printf("%.10f ", arr[i][j]);
    }
    printf("\n");
  }

  return arr;
}
