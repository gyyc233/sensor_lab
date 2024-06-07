#include "gaussian_blur/gaussian_blur.h"

GaussianBlur::GaussianBlur() {}

GaussianBlur::~GaussianBlur() {}

void GaussianBlur::gaussianBlur(const cv::Mat &srcImage, cv::Mat &dstImage,
                                int ksize, double sigma) const {}

void GaussianBlur::convolution(const cv::Mat &my_image, cv::Mat &result,
                               const std::vector<std::vector<double>> &kernel,
                               int ksize) const {
  const int channels = my_image.channels();
  int center = static_cast<int>(ksize / 2);

  // expand image boundary
  cv::Mat expand_image = cv::Mat::zeros(my_image.rows + center * 2,
                                        my_image.cols + center * 2, CV_8UC1);

  for (int i = 0; i < my_image.rows; i++) {
    for (int j = 0; j < my_image.cols; j++) {
      expand_image.at<uchar>(i + center, j + center) = my_image.at<uchar>(i, j);
    }
  }

  // convolution
  for (int row = center; row < expand_image.rows - center; row++) {
    for (int col = center; col < expand_image.cols - center; col++) {
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
      expand_image.at<uchar>(row, col) = tmp_convolution;
    }
  }

  // crop image, remove extra boundary
  result = cv::Mat::zeros(my_image.size(), CV_8UC1);
  for (int i = center; i < expand_image.rows - center; i++) {
    for (int j = center; j < expand_image.cols - center; j++) {
      result.at<uchar>(i - center, j - center) = expand_image.at<uchar>(i, j);
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
