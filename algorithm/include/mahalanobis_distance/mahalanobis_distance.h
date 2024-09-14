#ifndef __MAHALANOBIS_DISTANCE_H__
#define __MAHALANOBIS_DISTANCE_H__

#include <opencv2/opencv.hpp>
#include <vector>

namespace Algorithm {
class MahalanobisDistance {
public:
  MahalanobisDistance();
  ~MahalanobisDistance();

  double
  mahalDistanceOfTwoDistribution(const std::vector<cv::Point2f> &points1,
                                 const std::vector<cv::Point2f> &points2);

  double mahalDistance(const std::vector<cv::Point2f> &distribution,
                       const cv::Point2f &point);

private:
  void calcMeanXY(const std::vector<cv::Point2f> &points, double &mean_x,
                  double &mean_y);

  void calcCovariance(const std::vector<cv::Point2f> &points,
                      std::vector<double> &cov_vec);
};
} // namespace Algorithm

#endif
