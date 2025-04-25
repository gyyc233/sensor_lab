#ifndef FEATURE_TRACKER_H_
#define FEATURE_TRACKER_H_

#include <csignal>
#include <execinfo.h>
#include <iostream>
#include <queue>

#include <eigen3/Eigen/Dense>
#include <opencv2/opencv.hpp>

namespace sensor_lab {

using namespace std;
using namespace Eigen;

class FeatureTracker {
public:
  FeatureTracker();
  ~FeatureTracker();

  /// @brief 前端处理主流乘
  /// @param _img
  /// @param _cur_time
  void readImage(const cv::Mat &_img, double _cur_time);

  /// @brief 异常值剔除
  void rejectWithF();

  void setMask();

  /// @brief set camera distortion params
  /// @param params [k1,k2,k3,p1,p2]
  void setDistortionParams(const vector<double> &params);

  /// @brief 将点从图像平面提升到其投影光线
  /// @param p image coordinates
  /// @param P coordinates of the projective ray
  void liftProjective(const Eigen::Vector2d &p, Eigen::Vector3d &P);

  vector<cv::Point2f> prev_pts, cur_pts, forw_pts; // 像素坐标
  vector<cv::Point2f> prev_un_pts, cur_un_pts;     // 去畸变归一化坐标
  vector<cv::Point2f> pts_velocity; // 特征点速度(像素/帧)

  cv::Mat prev_img; // 上一帧图像
  cv::Mat cur_img;  // 当前帧图像
  cv::Mat
      forw_img; // 当前帧的灰度图像（经过预处理(畸变矫正)但尚未用于光流跟踪）,避免污染原始数据

  vector<int> ids;                      // 特征点唯一ID
  vector<int> track_cnt;                // 帧id?
  map<int, cv::Point2f> cur_un_pts_map; // ID到归一化坐标的映射
  map<int, cv::Point2f> prev_un_pts_map;
  double cur_time;
  double prev_time;
  cv::Mat mask; // 特征检测掩码，控制特征检测区域（避免在边缘或无效区域检测）
  int image_rows;
  int image_cols;

  vector<double> distortion_param; // [k1,k2,k3,p1,p2]
};

} // namespace sensor_lab

#endif
