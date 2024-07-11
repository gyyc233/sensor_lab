#include <Eigen/Dense>
#include <Eigen/QR>
#include <Eigen/SVD>
#include <opencv2/opencv.hpp>
#include <string>
#include <vector>

namespace Algorithm {
class Undistortion {
private:
  double k1_;
  double k2_;
  double k3_;
  double p1_;
  double p2_;

  double fx_; // fx=f/(Zc*dx); f:相机焦距; 1/dx:图像坐标系x方向每毫米像素数,
              // Zc:物体在相机坐标系下的深度
  double fy_; // fy=f/(Zc*dy); f:相机焦距; 1/dy:图像坐标系y方向每毫米像素数
  double cx_; // 图像坐标系原点在像素坐标系下的x坐标，单位:像素
  double cy_; // 图像坐标系原点在像素坐标系下的y坐标，单位:像素

  cv::Mat input_image_;        // including distortion image
  cv::Mat undistortion_image_; // undistortion image
public:
  Undistortion();
  ~Undistortion();

  /// @brief input params
  /// @param image_path image path
  /// @param intrinsics camera intrinsics array [fx, fy, cx, cy]
  /// @param distortion_params camera distortion array [k1,k2,k3,p1,p2]
  void inputParams(const std::string &image_path,
                   const std::vector<double> &intrinsics,
                   const std::vector<double> &distortion_params);

  bool run();

  void output(cv::Mat &undistorted_image, cv::Mat &distorted_image);
};
} // namespace Algorithm
