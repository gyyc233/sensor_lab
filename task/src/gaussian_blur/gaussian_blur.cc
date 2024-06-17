#include "gaussian_blur/gaussian_blur.h"

GaussianBlur::GaussianBlur() {}

GaussianBlur::~GaussianBlur() {}

void GaussianBlur::gaussianBlur(const cv::Mat &srcImage, cv::Mat &dstImage,
                                int ksize, double sigma) const {}

void GaussianBlur::convolution(const cv::Mat &my_image, cv::Mat &result,
                               const std::vector<std::vector<double>> &kernel,
                               int ksize) const {
  if (my_image.channels() != 1) {
    std::cout << "only accept single channel mat" << std::endl;
    return;
  }
  int center = static_cast<int>(ksize / 2);

  // expand image boundary
  cv::Mat expand_image;
  expandImageBoundary(my_image, expand_image, center);

  // convolution
  cv::Mat convolution_result_image =
      cv::Mat::zeros(expand_image.rows, expand_image.cols, CV_8UC1);
  for (int row = center; row < expand_image.rows - center; row++) {
    for (int col = center; col < expand_image.cols - center; col++) {
      // estimate kernel sum
      double tmp_convolution = 0;
      for (int i = 0; i < ksize; i++) {
        for (int j = 0; j < ksize; j++) {
          tmp_convolution +=
              kernel[i][j] *
              expand_image.at<uchar>(row - center + i, col - center + j);
          std::cout << kernel[i][j] * expand_image.at<uchar>(row - center + i,
                                                             col - center + j)
                    << ", ";
        }
      }
      std::cout << std::endl;
      std::cout << "index(row,col): " << row << ", " << col
                << ", tmp_convolution: " << tmp_convolution << std::endl;
      convolution_result_image.at<uchar>(row, col) = tmp_convolution;
    }
  }

  // crop image, remove extra boundary
  cropImageBoundary(convolution_result_image, result, center);
}

void GaussianBlur::expandImageBoundary(const cv::Mat &input, cv::Mat &output,
                                       size_t radius) const {
  assert(radius > 0);
  output =
      cv::Mat::zeros(input.rows + radius * 2, input.cols + radius * 2, CV_8UC1);
  for (int i = 0; i < input.rows; i++) {
    for (int j = 0; j < input.cols; j++) {
      output.at<uchar>(i + radius, j + radius) = input.at<uchar>(i, j);
    }
  }
}

void GaussianBlur::cropImageBoundary(const cv::Mat &input, cv::Mat &output,
                                     size_t radius) const {
  assert((input.rows - radius * 2) > 0 && (input.cols - radius * 2) > 0);
  output =
      cv::Mat::zeros(input.rows - radius * 2, input.cols - radius * 2, CV_8UC1);
  for (int i = radius; i < input.rows - radius; i++) {
    for (int j = radius; j < input.cols - radius; j++) {
      output.at<uchar>(i - radius, j - radius) = input.at<uchar>(i, j);
    }
  }
}

std::vector<std::vector<double>>
GaussianBlur::builtMeanKernel(int ksize) const {
  double element = 1.0 / (ksize * ksize);

  std::vector<std::vector<double>> arr(ksize, std::vector<double>(ksize, 0));
  for (int i = 0; i < arr.size(); i++) {
    for (int j = 0; j < arr[0].size(); j++) {
      arr[i][j] = element;
    }
  }

  printKernel(arr);

  return arr;
}

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
  printKernel(arr);

  return arr;
}

void GaussianBlur::printKernel(
    const std::vector<std::vector<double>> &kernel) const {
  printf("printf kernel, size: %ld \n", kernel.size());
  for (int i = 0; i < kernel.size(); i++) {
    for (int j = 0; j < kernel[0].size(); j++) {
      printf("%.10f ", kernel[i][j]);
    }
    printf("\n");
  }
}
