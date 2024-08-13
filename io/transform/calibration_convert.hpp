#ifndef __CALIBRATION_CONVERT_HPP__
#define __CALIBRATION_CONVERT_HPP__

#include <Eigen/Core>
#include <Eigen/Dense>
#include <iostream>
#include <opencv2/core/eigen.hpp>
#include <opencv2/opencv.hpp>
#include <vector>

/// @brief pixel coordination to camera frame
/// @param p pixel (u,v)
/// @param K camera intrinsics matrix [fx,0,cx;0,fy,cy;0,0,1]
/// @return normal camera coordination [x,y]
cv::Point2d pixel2CameraFrame(const cv::Point2d &p, const cv::Mat &K) {
  return cv::Point2d((p.x - K.at<double>(0, 2)) / K.at<double>(0, 0),
                     (p.y - K.at<double>(1, 2)) / K.at<double>(1, 1));
}

/// @brief camera coordination to pixel frame
/// @param p camera coordination [x,y,z]
/// @note ignore camera distortion
/// @param K camera intrinsics matrix [fx,0,cx;0,fy,cy;0,0,1]
/// @return pixel (u,v)
cv::Point2d cam2PixelFrame(const cv::Point3d &p, const cv::Mat &K) {
  assert(p.z != 0);
  cv::Point3d p_norm(p.x / p.z, p.y / p.z, p.z / p.z);
  return cv::Point2d((K.at<double>(0, 0) * p_norm.x + K.at<double>(0, 2)),
                     (K.at<double>(1, 1) * p_norm.y + K.at<double>(1, 2)));
}

/// @brief based on distortion param undistorted camera coordination
/// @param p camera coordination
/// @param params distortion params [k1,k2,k3,p1,p2]
/// @return undistorted camera coordination
cv::Point2d cameraCoordinationUndistortion(const cv::Point2d &p,
                                           const std::vector<double> &params) {
  double r = sqrt(p.x * p.x + p.y * p.y);

  double undistorted_image_x =
      p.x * (1 + params[0] * r * r + params[1] * r * r * r * r +
             params[2] * r * r * r * r * r * r) +
      2 * params[4] * p.x * p.y + params[5] * (r * r + 2 * p.x * p.x);
  double undistorted_image_y =
      p.y * (1 + params[0] * r * r + params[1] * r * r * r * r +
             params[2] * r * r * r * r * r * r) +
      2 * params[4] * (r * r + 2 * p.x * p.x) + params[5] * p.x * p.y;

  return cv::Point2d(undistorted_image_x, undistorted_image_y);
}

#endif
