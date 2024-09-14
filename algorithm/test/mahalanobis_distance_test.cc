#include "mahalanobis_distance/mahalanobis_distance.h"
#include <memory>
#include <opencv2/opencv.hpp>
#include <vector>

using namespace Algorithm;

void two_distribution();

int main() { two_distribution(); }

void two_distribution() {
  std::vector<cv::Point2f> points1;
  std::vector<cv::Point2f> points2;

  points1.emplace_back(2, 2);
  points1.emplace_back(2, 5);
  points1.emplace_back(6, 5);
  points1.emplace_back(7, 3);
  points1.emplace_back(4, 7);
  points1.emplace_back(6, 4);
  points1.emplace_back(5, 3);
  points1.emplace_back(4, 6);
  points1.emplace_back(2, 5);
  points1.emplace_back(1, 3);

  points2.emplace_back(6, 5);
  points2.emplace_back(7, 4);
  points2.emplace_back(8, 7);
  points2.emplace_back(5, 6);
  points2.emplace_back(5, 4);

  MahalanobisDistance mahal_distance;
  double distance_1 =
      mahal_distance.mahalDistanceOfTwoDistribution(points1, points2);
  std::cout << "ex 1. mahalanobis distance: " << distance_1 << std::endl;

  double distance_2 = mahal_distance.mahalDistance(points1, cv::Point2f(0, 0));
  std::cout << "ex 2. mahalanobis distance: " << distance_2 << std::endl;
}
