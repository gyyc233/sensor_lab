#ifndef VINS_ESTIMATOR_INITIAL_IMU_CAMERA_EX_ROTATION
#define VINS_ESTIMATOR_INITIAL_IMU_CAMERA_EX_ROTATION

#include "utility/utility.h"
#include <eigen3/Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <vector>

/* This class help you to calibrate extrinsic rotation between imu and camera
 * when your totally don't konw the extrinsic parameter */
// 估计imu与相机之间的外参旋转

namespace sensor_lab {
class InitialEXRotation {
public:
  InitialEXRotation();

  /// @brief 使用视觉点对和 IMU 提供的旋转增量来估计相机与 IMU 之间的旋转外参
  /// @param corres 视觉帧中匹配的 3D 点对（可能是三角化得到的）
  /// @param delta_q_imu IMU 提供的两帧之间的旋转四元数
  /// @param calib_ric_result 输出估计得到的旋转矩阵
  /// @return
  bool CalibrationExRotation(
      std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> corres,
      Eigen::Quaterniond delta_q_imu, Eigen::Matrix3d &calib_ric_result);

private:
  /// @brief 使用点对计算两帧之间的相对旋转矩阵
  /// @param corres
  /// @return
  Eigen::Matrix3d solveRelativeR(
      const std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> &corres);

  /// @brief 对给定的旋转和平移进行三角化，统计有多少点在相机前方（正深度）
  double testTriangulation(const std::vector<cv::Point2f> &l,
                           const std::vector<cv::Point2f> &r,
                           cv::Mat_<double> R, cv::Mat_<double> t);

  /// @brief
  void decomposeE(cv::Mat E, cv::Mat_<double> &R1, cv::Mat_<double> &R2,
                  cv::Mat_<double> &t1, cv::Mat_<double> &t2);
  int frame_count; // 外参估计帧数

  std::vector<Eigen::Matrix3d> Rc;   // 每一帧的视觉旋转估计
  std::vector<Eigen::Matrix3d> Rimu; // 每一帧的IMU旋转估计
  std::vector<Eigen::Matrix3d>
      Rc_g; // 通过imu旋转估计与imu-camera外参估计的相机帧间旋转估计
  Eigen::Matrix3d ric; // 相机到IMU的旋转矩阵
  int window_size = 10;
};
} // namespace sensor_lab

#endif
