#pragma once

#include <opencv2/opencv.hpp>
#include <vector>

class GaussianBlur {
public:
  GaussianBlur();
  ~GaussianBlur();

  void gaussianBlur(const cv::Mat &srcImage, cv::Mat &dstImage, int ksize,
                    double sigma) const;

  /// @brief appoint kernel size, based on kernel built gaussian kernel
  /// @note appoint sigma to make kernel size
  ///       double sigma= 0.7;
  ///       int ksize = 2* ceil(3*sigma)-1; //ceil函数向上取整，且需要保证为奇数
  /// @param ksize size of kernel
  /// @param sigma Gaussian standard deviation, default sigma =
  /// 0.3*((ksize-1)*0.5 - 1) + 0.8
  /// @return gaussian kernel, 2 dimension array
  std::vector<std::vector<double>> builtGaussianKernel(int ksize,
                                                       double sigma) const;

  std::vector<std::vector<double>> builtMeanKernel(int ksize) const;

  void convolution(const cv::Mat &my_image, cv::Mat &result,
                   const std::vector<std::vector<double>> &kernel,
                   int ksize) const;

private:
  void expandImageBoundary(const cv::Mat &input, cv::Mat &output,
                           size_t radius) const;

  void cropImageBoundary(const cv::Mat &input, cv::Mat &output,
                         size_t radius) const;

  void printKernel(const std::vector<std::vector<double>> &kernel) const;
};
