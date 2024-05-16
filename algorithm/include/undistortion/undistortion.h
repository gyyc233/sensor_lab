#include <Eigen/Dense>
#include <Eigen/QR>
#include <Eigen/SVD>
#include <opencv2/opencv.hpp>
#include <string>
#include <vector>

namespace Algorithm {
class Undistortion {
private:
  int first_x = 0;
  int first_y = 0;
  Eigen::MatrixXd k1;
  Eigen::MatrixXd k2;

  std::vector<int> normalized_x_;
  std::vector<int> normalized_y_;
  int normalization_ref_x_;
  int normalization_ref_y_;
  int ref_distance_;

public:
  Undistortion();
  ~Undistortion();

  int detectConners(const std::string &image_path, std::vector<int> &points_x,
                    std::vector<int> &points_y);

  int initFactor(std::vector<int> &source_x, std::vector<int> &source_y,
                 std::vector<int> &target_x, std::vector<int> &target_y);

  int positiveTransform(std::vector<int> &source_x, std::vector<int> &source_y);

  int normalization(std::vector<int> &source_x, std::vector<int> &source_y);

  int process(std::vector<int> &source_x, std::vector<int> &source_y,
              int ref_x = 160, int ref_y = 96);

  int reshapeOriginPoints(std::vector<int> &source_x, std::vector<int> &source_y);
};
} // namespace Algorithm
