#include "feature_detect.h"
#include <nmmintrin.h>
#include <opencv2/opencv.hpp>
#include <string>
#include <vector>

namespace SensorLab {
typedef std::vector<uint32_t> DescType;

class OrbMy : public FeatureDetect {
public:
  OrbMy();
  ~OrbMy();

  void inputParams(const char *left_img, const char *right_img);

  void initialization() override;

  void run() override;

  /// @brief compute descriptor of orb key points
  /// @param img input image
  /// @param key_points detected fast key points
  /// @param descriptors descriptors
  void computeORB(const cv::Mat &img, std::vector<cv::KeyPoint> &key_points,
                  std::vector<DescType> &descriptors);

  /// @brief brute-force match two sets of descriptors
  /// @param desc1 the first descriptor
  /// @param desc2 the second descriptor
  /// @param matches matches of two images
  void BfMatch(const std::vector<DescType> &desc1,
               const std::vector<DescType> &desc2,
               std::vector<cv::DMatch> &matches);

private:
  cv::Mat img_l_;
  cv::Mat img_r_;

  int ORB_pattern_[256 * 4];
};
} // namespace SensorLab
