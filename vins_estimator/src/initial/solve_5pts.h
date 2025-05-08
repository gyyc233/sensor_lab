#ifndef VINS_ESTIMATOR_SOLVE_5PTS
#define VINS_ESTIMATOR_SOLVE_5PTS

#include <eigen3/Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <vector>

namespace sensor_lab {

// 通过两帧之间的 3D 点对应关系来估计相机的相对运动
class MotionEstimator {
public:
  /// @brief 给定一组 3D 点的匹配对，计算两帧之间的相对旋转 R 和平移 T
  /// @param corres 3D 点的匹配对, 实际只用了前2维
  /// @param R 两帧之间的相对旋转
  /// @param T 两帧之间的相对平移
  /// @return
  bool solveRelativeRT(
      const std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> &corres,
      Eigen::Matrix3d &R, Eigen::Vector3d &T);

private:
  /// @brief 使用给定的旋转 R 和平移 t，评估左右图像中的 2D
  /// 点是否能成功三角化为　 3D 点 原代码中放到 initial_ex_rotation中实现
  /// @param l 左图像中的 2D 点
  /// @param r 右图像中的 2D 点
  /// @param R 预测的旋转矩阵
  /// @param t 预测的平移向量
  /// @return 表示三角化后内点数量
  double testTriangulation(const std::vector<cv::Point2f> &l,
                           const std::vector<cv::Point2f> &r,
                           cv::Mat_<double> R, cv::Mat_<double> t);

  /// @brief 使用给定的本质矩阵 E，计算两帧之间的相对旋转 R 和平移
  /// t，两种旋转和两种平移）原代码中放到 initial_ex_rotation中实现
  void decomposeE(cv::Mat E, cv::Mat_<double> &R1, cv::Mat_<double> &R2,
                  cv::Mat_<double> &t1, cv::Mat_<double> &t2);
};
} // namespace sensor_lab

#endif
