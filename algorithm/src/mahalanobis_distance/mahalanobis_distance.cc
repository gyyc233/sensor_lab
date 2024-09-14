#include "mahalanobis_distance/mahalanobis_distance.h"
#include <iostream>

using namespace Algorithm;

MahalanobisDistance::MahalanobisDistance() {}

MahalanobisDistance::~MahalanobisDistance() {}

void MahalanobisDistance::calcMeanXY(const std::vector<cv::Point2f> &points,
                                     double &mean_x, double &mean_y) {
  mean_x = 0;
  mean_y = 0;
  for (size_t i = 0; i < points.size(); i++) {
    mean_x += points[i].x;
    mean_y += points[i].y;
  }

  mean_x = mean_x / points.size();
  mean_y = mean_y / points.size();
}

void MahalanobisDistance::calcCovariance(const std::vector<cv::Point2f> &points,
                                         std::vector<double> &cov_vec) {
  assert(points.size() > 1);

  double mean_x = 0, mean_y = 0;
  calcMeanXY(points, mean_x, mean_y);

  cov_vec.resize(3, 0);
  std::vector<double> &cov = cov_vec;
  for (const auto &point : points) {
    cov[0] += std::pow(point.x - mean_x, 2);
    cov[2] += std::pow(point.y - mean_y, 2);
    cov[1] += (point.x - mean_x) * (point.y - mean_y);
  }

  double inv_size = 1.0 / (points.size() - 1);
  cov[0] *= inv_size;
  cov[1] *= inv_size;
  cov[2] *= inv_size;
}

double MahalanobisDistance::mahalDistanceOfTwoDistribution(
    const std::vector<cv::Point2f> &points1,
    const std::vector<cv::Point2f> &points2) {
  std::vector<double> cov_a;
  std::vector<double> cov_b;

  calcCovariance(points1, cov_a);
  calcCovariance(points2, cov_b);

  // pooled variance 合并方差
  // https://people.revoledu.com/kardi/tutorial/Similarity/MahalanobisDistance.html
  double inv_all = 1.0 / (points1.size() + points2.size());
  for (size_t i = 0; i < cov_a.size(); ++i) {
    cov_a[i] = cov_a[i] * points1.size() * inv_all +
               cov_b[i] * points2.size() * inv_all;
  }

  // estimate covariance(2*2 matrix) inverse
  double ad_bc = (cov_a[0] * cov_a[2] - cov_a[1] * cov_a[1]);
  if (std::fabs(ad_bc) <= std::numeric_limits<double>::epsilon()) {
    double mean_a_x, mean_a_y;
    double mean_b_x, mean_b_y;
    calcMeanXY(points1, mean_a_x, mean_a_y);
    calcMeanXY(points2, mean_b_x, mean_b_y);

    double value_x = mean_a_x - mean_b_x;
    double value_y = mean_a_y - mean_b_y;
    return std::sqrt(value_x * value_x + value_y * value_y);
  } else {
    double inv_ad_bc = 1.0 / (cov_a[0] * cov_a[2] - cov_a[1] * cov_a[1]);
    std::vector<double> inv_cov(4);
    inv_cov[0] = cov_a[2] * inv_ad_bc;
    inv_cov[1] = -cov_a[1] * inv_ad_bc;
    inv_cov[2] = inv_cov[1];
    inv_cov[3] = cov_a[0] * inv_ad_bc;

    double mean_a_x, mean_a_y;
    double mean_b_x, mean_b_y;
    calcMeanXY(points1, mean_a_x, mean_a_y);
    calcMeanXY(points2, mean_b_x, mean_b_y);

    // estimate mahalanobis distance
    double value_x = mean_a_x - mean_b_x;
    double value_y = mean_a_y - mean_b_y;

    double temp1 = value_x * inv_cov[0] + value_y * inv_cov[1];
    double temp2 = value_x * inv_cov[2] + value_y * inv_cov[3];

    double dist = std::sqrt(temp1 * value_x + temp2 * value_y);

    return dist;
  }
}

double
MahalanobisDistance::mahalDistance(const std::vector<cv::Point2f> &distribution,
                                   const cv::Point2f &point) {

  std::vector<double> cov;
  calcCovariance(distribution, cov);

  double ad_bc = (cov[0] * cov[2] - cov[1] * cov[1]);
  if (std::fabs(ad_bc) <= std::numeric_limits<double>::epsilon()) {
    double mean_x, mean_y;
    calcMeanXY(distribution, mean_x, mean_y);

    double value_x = point.x - mean_x;
    double value_y = point.y - mean_y;
    return std::sqrt(value_x * value_x + value_y * value_y);
  } else {
    double inv_ad_bc = 1.0 / (cov[0] * cov[2] - cov[1] * cov[1]);
    std::vector<double> inv_cov(4);
    inv_cov[0] = cov[2] * inv_ad_bc;
    inv_cov[1] = -cov[1] * inv_ad_bc;
    inv_cov[2] = inv_cov[1];
    inv_cov[3] = cov[0] * inv_ad_bc;

    double mean_x, mean_y;
    calcMeanXY(distribution, mean_x, mean_y);

    double value_x = point.x - mean_x;
    double value_y = point.y - mean_y;

    double temp1 = value_x * inv_cov[0] + value_y * inv_cov[1];
    double temp2 = value_x * inv_cov[2] + value_y * inv_cov[3];

    double dist = std::sqrt(temp1 * value_x + temp2 * value_y);

    return dist;
  }
}
