#include <Eigen/Dense>
#include <Eigen/QR>
#include <Eigen/SVD>
#include <opencv2/opencv.hpp>
#include <string>
#include <vector>

class ORB_CV {
public:
  void inputParams(const char *left_img, const char *right_img);
};
