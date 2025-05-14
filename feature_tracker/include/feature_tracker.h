#ifndef FEATURE_TRACKER_H
#define FEATURE_TRACKER_H

#include <csignal>
#include <execinfo.h>
#include <iostream>
#include <queue>

#include "pinhole_camera.h"
#include "tic_toc.h"
#include <eigen3/Eigen/Dense>
#include <opencv2/opencv.hpp>

namespace sensor_lab {

using namespace std;
using namespace Eigen;

class FeatureTracker {
public:
  FeatureTracker();
  ~FeatureTracker();

  /// @brief 前端处理主流程
  /// @param _img
  /// @param _cur_time
  void readImage(const cv::Mat &_img, double _cur_time);

  /// @brief 异常值剔除
  void rejectWithF();

  void setMask();

  void addPoints();

  /// @brief 计算特征点速度
  void undistortedPoints();

  /// @brief 根据状态向量 status 剔除跟踪失败的特征点及相关数据
  /// @param v
  /// @param status
  void reduceVector(vector<int> &v, vector<uchar> status);

  void reduceVector(vector<cv::Point2f> &v, vector<uchar> status);

  /// @brief set max feature points number
  /// @param max_corners_count
  void setMaxCornersCount(int max_corners_count);

  void setCamera(sensor_lab::PinholeCameraPtr camera_ptr);

  /// @brief 为特征点分配唯一的 ID
  /// @param i 当前要处理的特征点在其对应的特征列表中的索引
  /// @return
  bool updateID(unsigned int i);

  vector<cv::Point2f> n_pts;
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
  int max_corners_count_;

  sensor_lab::PinholeCameraPtr camera_ptr_;

  static int n_id; // 生成特征点id
};

} // namespace sensor_lab

#endif
