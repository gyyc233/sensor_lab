#include "tps/tps.h"
#include <opencv2/imgcodecs.hpp>

using namespace Algorithm;

TPSOperator::TPSOperator() {}

TPSOperator::~TPSOperator() {}

double TPSOperator::tps_basis(double r) {
  if (abs(r) <= 10e-6) {
    return 0.0;
  } else {
    // based e -->log
    // based 10 -->log10()
    // 自定义以m为底，求log n的值
    // double a=log(n)/log(m);
    return (r * log(r));
  }
}

void TPSOperator::calculate_K(const std::vector<cv::Point2f> &points,
                              cv::Mat &K) {
  if (K.empty()) {
    K.create(points.size(), points.size(), CV_32FC1);
  } else if (K.rows != points.size() || K.cols != points.size()) {
    cv::resize(K, K, cv::Size(points.size(), points.size()));
  }

  for (int i = 0; i < points.size(); i++) {
    for (int j = i; j < points.size(); j++) {
      cv::Point2f diff_p = points[i] - points[j];
      double diff = diff_p.x * diff_p.x + diff_p.y * diff_p.y;
      double basis = tps_basis(diff);
      K.ptr<float>(i)[j] = static_cast<float>(basis);
      K.ptr<float>(j)[i] = static_cast<float>(basis);
    }
  }
}

void TPSOperator::calculate_L(const std::vector<cv::Point2f> &points,
                              cv::Mat &L) {
  int n = points.size();
  cv::Mat L_tmp = cv::Mat::zeros(n + 3, n + 3, CV_32FC1);

  cv::Mat P = cv::Mat::ones(n, 3, CV_32FC1);
  for (int i = 0; i < n; i++) {
    P.ptr<float>(i)[1] = points[i].x;
    P.ptr<float>(i)[2] = points[i].y;
  }

  cv::Mat P_reversal;
  cv::transpose(P, P_reversal);

  cv::Mat K;
  calculate_K(points, K);

  K.copyTo(L_tmp(cv::Rect(0, 0, n, n)));
  P.copyTo(L_tmp(cv::Rect(n, 0, 3, n)));
  P_reversal.copyTo(L_tmp(cv::Rect(0, n, n, 3)));

  L = L_tmp.clone();
}

void TPSOperator::calculate_W(const std::vector<cv::Point2f> &points_source,
                              const std::vector<cv::Point2f> &points_target,
                              cv::Mat &W) {
  cv::Mat L;
  calculate_L(points_source, L);

  int n = points_target.size();
  cv::Mat Y = cv::Mat::zeros(n + 3, 2, CV_32FC1);

  for (int i = 0; i < n; i++) {
    Y.ptr<float>(i)[0] = points_target[i].x;
    Y.ptr<float>(i)[1] = points_target[i].y;
  }

  std::cout << "solve result: " << cv::solve(L, Y, W, cv::DECOMP_SVD)
            << std::endl;
  std::cout << "W:\n" << W << std::endl;
}

cv::Point2f
TPSOperator::tps_transformation(const std::vector<cv::Point2f> &points_source,
                                const cv::Point2f &source_point,
                                const cv::Mat &W) {
  float a1_x = W.at<float>(W.rows - 3, 0);
  float ax_x = W.at<float>(W.rows - 2, 0);
  float ay_x = W.at<float>(W.rows - 1, 0);

  float a1_y = W.at<float>(W.rows - 3, 1);
  float ax_y = W.at<float>(W.rows - 2, 1);
  float ay_y = W.at<float>(W.rows - 1, 1);

  float affine_x = a1_x + ax_x * source_point.x + ay_x * source_point.y;
  float affine_y = a1_y + ax_y * source_point.x + ay_y * source_point.y;

  float nonrigid_x = 0;
  float nonrigid_y = 0;

  for (int i = 0; i < points_source.size(); i++) {
    cv::Point2f diff_p = points_source[i] - source_point;
    double diff = diff_p.x * diff_p.x + diff_p.y * diff_p.y;
    double basis = tps_basis(diff);

    nonrigid_x += W.at<float>(i, 0) * basis;
    nonrigid_y += W.at<float>(i, 1) * basis;
  }

  cv::Point point_target;
  point_target.x = affine_x + nonrigid_x;
  point_target.y = affine_y + nonrigid_y;

  return std::move(point_target);
}

void TPSOperator::tps_map(const std::vector<cv::Point2f> &points_source,
                          const cv::Mat &W, const int rows, const int cols,
                          cv::Mat &target_x, cv::Mat &target_y) {
  cv::Mat map_x(rows, cols, CV_32FC1);
  cv::Mat map_y(rows, cols, CV_32FC1);

  for (int i = 0; i < rows; i++) {
    for (int j = 0; j < cols; j++) {
      cv::Point2f pixel =
          tps_transformation(points_source, cv::Point2f(float(j), float(i)), W);
      map_x.at<float>(i, j) = pixel.x;
      map_y.at<float>(i, j) = pixel.y;
    }
  }

  map_x.copyTo(target_x);
  map_y.copyTo(target_y);
}
