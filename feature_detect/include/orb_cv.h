#include "feature_detect.h"
#include "my_time.h"
#include <Eigen/Dense>
#include <Eigen/QR>
#include <Eigen/SVD>
#include <opencv2/opencv.hpp>
#include <string>
#include <vector>

namespace SensorLab {
class ORB_CV : public FeatureDetect {
public:
  ORB_CV();

  ~ORB_CV();

  void inputParams(const char *left_img, const char *right_img);

  void initialization() override;

  void run() override;

private:
  cv::Mat img_l_;
  cv::Mat img_r_;

  cv::Ptr<cv::FeatureDetector> detector_;
  cv::Ptr<cv::DescriptorExtractor> descriptor_;
  cv::Ptr<cv::DescriptorMatcher> matcher_;

  CostMillisecond cost_;
};
} // namespace SensorLab
