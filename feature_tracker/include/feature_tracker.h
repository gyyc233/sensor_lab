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

  /// @brief 前端处理主流程
  /// @param _img
  /// @param _cur_time
  void readImage(const cv::Mat &_img, double _cur_time);

  /// @brief 异常值剔除
  void rejectWithF();

  void setMask();

  void addPoints();

  /// @brief set camera distortion params
  /// @param params [k1,k2,k3,p1,p2]
  void setDistortionParams(const vector<double> &params);

  /// @brief set camera intrinsic matrix
  /// @param camera_k [f_x,f_y,c_x,x_y]
  void setCameraKMatrix(const vector<double> &camera_k);

  /// @brief 将2D图像像素坐标 p 转换为3D归一化射线向量 P（单位球面上的方向向量）
  /// @param p image coordinates
  /// @param P coordinates of the projective ray
  void liftProjective(const Eigen::Vector2d &p, Eigen::Vector3d &P);

  /// @brief 将畸变应用于归一化平面坐标b并计算jacobian
  /// @param p_u 归一化平面坐标x
  /// @param d_u 归一化平面坐标y
  /// @param J
  void undistortionDistortion(const Eigen::Vector2d &p_u, Eigen::Vector2d &d_u,
                              Eigen::Matrix2d &J);

  /// @brief Apply distortion to input point (from the normalised plane)
  /// @param p_u
  /// @param d_u
  void distortion(const Eigen::Vector2d &p_u, Eigen::Vector2d &d_u) const;

  /// @brief 计算特征点速度
  void undistortedPoints();

  /// @brief 根据状态向量 status 剔除跟踪失败的特征点及相关数据
  /// @param v
  /// @param status
  void reduceVector(vector<int> &v, vector<uchar> status);

  void reduceVector(vector<cv::Point2f> &v, vector<uchar> status);

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

  vector<double> distortion_param; // [k1,k2,k3,p1,p2]
  vector<double> camera_K;         // [f_x,f_y,c_x,x_y]
};

} // namespace sensor_lab

#endif
