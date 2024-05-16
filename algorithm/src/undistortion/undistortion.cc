#include "undistortion/undistortion.h"
#include <glog/logging.h>

namespace Algorithm {
Undistortion::Undistortion() {
  normalization_ref_x_=80;
  normalization_ref_y_=48;
  ref_distance_=32;
}

Undistortion::~Undistortion() {}

int Undistortion::detectConners(const std::string &image_path,
                                std::vector<int> &points_x,
                                std::vector<int> &points_y) {
  cv::Mat img_gary = cv::imread(image_path, 0);
  std::cout << "image info: " << img_gary.rows << ", " << img_gary.cols
            << std::endl;
  int conners_num = 0;
  for (int row = 1; row < img_gary.rows - 1; row++) {
    for (int col = 1; col < img_gary.cols - 1; col++) {
      if (img_gary.at<uchar>(row - 1, col - 1) > 21 &&
          img_gary.at<uchar>(row - 1, col) > 21 &&
          img_gary.at<uchar>(row - 1, col + 1) > 21 &&
          img_gary.at<uchar>(row, col - 1) > 21 &&
          img_gary.at<uchar>(row, col) > 21 &&
          img_gary.at<uchar>(row, col + 1) > 21 &&
          img_gary.at<uchar>(row + 1, col - 1) > 21 &&
          img_gary.at<uchar>(row + 1, col) > 12 &&
          img_gary.at<uchar>(row + 1, col + 1) > 21) {
        cv::Point2f p;
        p.x = col;
        p.y = row;

        if (conners_num == 0) {
          first_x = col;
          first_y = row;
        }

        points_x.push_back(col);
        points_y.push_back(row);
        conners_num++;
      }
    }
  }

  std::cout << "find conners number: " << conners_num << std::endl;
  return 0;
}

int Undistortion::initFactor(std::vector<int> &source_x,
                             std::vector<int> &source_y,
                             std::vector<int> &target_x,
                             std::vector<int> &target_y) {
  int points_size = source_x.size();
  Eigen::MatrixXd src_x(points_size, 1);
  Eigen::MatrixXd src_y(points_size, 1);
  Eigen::MatrixXd u(points_size, 6);

  for (int i = 0; i < points_size; i++) {
    src_x(i, 0) = source_x[i];
    src_y(i, 0) = source_y[i];

    u(i, 0) = 1;
    u(i, 1) = target_x[i];
    u(i, 2) = target_y[i];
    u(i, 3) = target_x[i] * target_x[i];
    u(i, 4) = target_y[i] * target_y[i];
    u(i, 5) = target_x[i] * target_y[i];
  }

  std::cout << "X:\n" << src_x << std::endl;
  std::cout << "Y:\n" << src_y << std::endl;
  std::cout << "U:\n" << u << std::endl;

  // calculate K1 K2
  // Eigen::MatrixXd k1(6,1);
  // Eigen::MatrixXd k2(6,1);

  k1 = (u.transpose() * u).inverse() * u.transpose() * src_x;
  k2 = (u.transpose() * u).inverse() * u.transpose() * src_y;

  std::cout << "k1:\n" << k1 << std::endl;
  std::cout << "k2:\n" << k2 << std::endl;

  return 0;
}

int Undistortion::positiveTransform(std::vector<int> &source_x,
                                    std::vector<int> &source_y) {
  int points_size = source_x.size();
  Eigen::MatrixXd u(points_size, 6);

  for (int i = 0; i < points_size; i++) {
    u(i, 0) = 1;
    u(i, 1) = source_x[i];
    u(i, 2) = source_y[i];
    u(i, 3) = source_x[i] * source_x[i];
    u(i, 4) = source_y[i] * source_y[i];
    u(i, 5) = source_x[i] * source_y[i];
  }
  std::cout << "U:\n" << u << std::endl;
  Eigen::MatrixXd predicated_x(points_size, 1);
  Eigen::MatrixXd predicated_y(points_size, 1);
  for (int i = 0; i < points_size; i++) {
    predicated_x(i, 0) = u(i, 0) * k1(0, 0) + u(i, 1) * k1(1, 0) +
                         u(i, 2) * k1(2, 0) + u(i, 3) * k1(3, 0) +
                         u(i, 4) * k1(4, 0) + u(i, 5) * k1(5, 0);
    predicated_y(i, 0) = u(i, 0) * k2(0, 0) + u(i, 1) * k2(1, 0) +
                         u(i, 2) * k2(2, 0) + u(i, 3) * k2(3, 0) +
                         u(i, 4) * k2(4, 0) + u(i, 5) * k2(5, 0);
  }

  std::cout << "predicated_x:\n" << predicated_x << std::endl;
  std::cout << "predicated_y:\n" << predicated_y << std::endl;

  return 0;
}

int Undistortion::normalization(std::vector<int> &source_x,
                                std::vector<int> &source_y) {
  int x_avg, y_avg;
  for (int i = 0; i < source_x.size(); i++) {
    x_avg += source_x[i];
    y_avg += source_y[i];
  }
  x_avg /= source_x.size();
  y_avg /= source_x.size();

  std::vector<int> normal_x, normal_y;
  for (int i = 0; i < source_x.size(); i++) {
    normal_x.push_back(source_x[i] - x_avg);
    normal_y.push_back(source_y[i] - y_avg);
    std::cout<<normal_x.back()<<", "<<normal_y.back()<<std::endl;
  }

  double max_distance_x = -1;
  double max_distance_y = -1;
  for (int i = 0; i < normal_x.size(); i++) {
    max_distance_x =
        max_distance_x > abs(normal_x[i]) ? max_distance_x : abs(normal_x[i]);
    max_distance_y =
        max_distance_y > abs(normal_y[i]) ? max_distance_y : abs(normal_y[i]);
  }

  normalization_ref_x_=max_distance_x;
  normalization_ref_y_=max_distance_y;
  std::cout << "normalization_ref_x_: " << ++normalization_ref_x_
            << ", normalization_ref_y_: " << ++normalization_ref_y_ << std::endl;

  normalized_x_=std::move(normal_x);
  normalized_y_=std::move(normal_y);
  return 0;
}

int Undistortion::reshapeOriginPoints(std::vector<int> &source_x, std::vector<int> &source_y){
  
}

}; // namespace Algorithm
