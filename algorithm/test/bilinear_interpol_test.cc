#include <iostream>
#include <opencv2/opencv.hpp>
#include <vector>

// reference https://zhuanlan.zhihu.com/p/513569382

void bilinearInterpol(cv::Mat src_image, double scale_x, double scale_y,
                      cv::Mat &dst_image) {
  for (int y = 0; y < dst_image.rows; y++) {
    for (int x = 0; x < dst_image.cols; x++) {

      // (x,j)为目标图像坐标
      // (px,py)原图坐标
      int px = (int)(x / scale_x);
      int py = (int)(y / scale_y);

      if (px >= src_image.cols - 1 || py >= src_image.rows - 1)
        break;

      // check y2-y1=1; x2-x1=1
      double fx1 = (double)x / (double)scale_x - (double)px; // x-x1
      double fx2 = 1 - fx1; // x2 - x = x1 + 1 - x =  1 - (x - x1)
      double fy1 = (double)y / (double)scale_y - (double)py; // y-y1
      double fy2 = 1 - fy1; // y2 - y = y1 + 1 - y =  1 - (y - y1)

      double w1 = fx2 * fy2;
      double w2 = fx1 * fy2;
      double w3 = fx2 * fy1;
      double w4 = fx1 * fy1;

      cv::Vec3b p1 = src_image.at<cv::Vec3b>(py, px);
      cv::Vec3b p2 = src_image.at<cv::Vec3b>(py, px + 1);
      cv::Vec3b p3 = src_image.at<cv::Vec3b>(py + 1, px);
      cv::Vec3b p4 = src_image.at<cv::Vec3b>(py + 1, px + 1);
      dst_image.at<cv::Vec3b>(y, x) = w1 * p1 + w2 * p2 + w3 * p3 + w4 * p4;
    }
  }
}

int main(int argc, char **argv) {
  cv::Mat image = cv::imread("./data/lena.png");
  if (image.empty()) {
    std::cout << "failed to read image" << std::endl;
    return 0;
  }

  double scale_x = 3.0;
  double scale_y = 3.0;

  // reference resize
  cv::Mat ref_bilinear_resize;
  cv::resize(image, ref_bilinear_resize, cv::Size(0, 0), scale_x, scale_y,
             cv::INTER_LINEAR);

  // 自定义resize函数实现，双线性插值
  int result_h = static_cast<int>(image.rows * scale_x);
  int result_w = static_cast<int>(image.cols * scale_y);
  cv::Mat result_im =
      cv::Mat::zeros(cv::Size(result_w, result_h), image.type());

  if (scale_x == 1 || scale_y == 1) {
    std::cout << "scale = 1" << std::endl;
    return 0;
  }

  bilinearInterpol(image, scale_x, scale_y, result_im);
  cv::Mat check_im;
  /// Create Windows
  namedWindow("Original Image", cv::WINDOW_AUTOSIZE);
  namedWindow("result Image", cv::WINDOW_AUTOSIZE);

  /// Show stuff
  imshow("Original Image", image);
  imshow("result Image", result_im);
  cv::waitKey(0);

  return 0;
}
