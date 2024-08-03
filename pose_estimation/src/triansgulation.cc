#include "triansgulation.h"
#include <iostream>

using namespace SensorLab;

Triansgulation::Triansgulation() {
  std::cout << "construct Triansgulation" << std::endl;
}

Triansgulation::~Triansgulation() {
  std::cout << "destruct Triansgulation" << std::endl;
}

void Triansgulation::initialization() { std::cout << "todo" << std::endl; }

void Triansgulation::run() { std::cout << "todo" << std::endl; }

inline cv::Scalar get_color(float depth) {
  float up_th = 50, low_th = 10, th_range = up_th - low_th;
  if (depth > up_th)
    depth = up_th;
  if (depth < low_th)
    depth = low_th;
  return cv::Scalar(255 * depth / th_range, 0, 255 * (1 - depth / th_range));
}

void Triansgulation::triangulation(const std::vector<cv::KeyPoint> &keypoint_l,
                                   const std::vector<cv::KeyPoint> &keypoint_r,
                                   const std::vector<cv::DMatch> &matches,
                                   const cv::Mat &R, const cv::Mat &t,
                                   std::vector<cv::Point3d> &points) {
  keypoint_l_ = keypoint_l;
  keypoint_r_ = keypoint_r;
  matches_ = matches;
  frame_rotation_ = R;
  frame_translation_ = t;

  // 创建两帧相机的投影矩阵
  cv::Mat T1 = (cv::Mat_<float>(3, 4) << 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0);
  cv::Mat T2 = (cv::Mat_<float>(3, 4) << R.at<double>(0, 0), R.at<double>(0, 1),
                R.at<double>(0, 2), t.at<double>(0, 0), R.at<double>(1, 0),
                R.at<double>(1, 1), R.at<double>(1, 2), t.at<double>(1, 0),
                R.at<double>(2, 0), R.at<double>(2, 1), R.at<double>(2, 2),
                t.at<double>(2, 0));

  // convert pixel to camera coordinates
  std::vector<cv::Point2f> camear_points_l, camear_points_r;
  for (size_t i = 0; i < matches.size(); i++) {
    camear_points_l.push_back(
        pixel2cam(keypoint_l[matches[i].queryIdx].pt, camera_matrix_));
    camear_points_r.push_back(
        pixel2cam(keypoint_r[matches[i].trainIdx].pt, camera_matrix_));
  }

  // each 3d point format: 4*1, based on camera coordination
  // 输出的3D坐标是齐次坐标，共四个维度，因此需要将前三个维度除以第四个维度以得到非齐次坐标xyz
  cv::Mat triangulation_result;
  cv::triangulatePoints(T1, T2, camear_points_l, camear_points_r,
                        triangulation_result);

  // 转换为非齐次坐标
  for (int i = 0; i < triangulation_result.cols; i++) {
    cv::Mat x = triangulation_result.col(i);
    x /= x.at<float>(3, 0); // 归一化
    cv::Point3d p(x.at<float>(0, 0), x.at<float>(1, 0), x.at<float>(2, 0));
    points_.push_back(p);
  }

  points = points_;
  std::cout << "3D points: \n" << points << std::endl;
}

void Triansgulation::inputCameraIntrinsics(
    const std::vector<double> &intrinsics) {
  camera_matrix_ = (cv::Mat_<double>(3, 3) << intrinsics[0], 0, intrinsics[2],
                    0, intrinsics[1], intrinsics[3], 0, 0, 1);
}

cv::Point2f Triansgulation::pixel2cam(const cv::Point2d &p, const cv::Mat &K) {
  return cv::Point2f((p.x - K.at<double>(0, 2)) / K.at<double>(0, 0),
                     (p.y - K.at<double>(1, 2)) / K.at<double>(1, 1));
}

void Triansgulation::checkReProject(cv::Mat &img_l, cv::Mat &img_r) {
  // 验证三角化点与特征点的重投影关系
  cv::Mat img_l_plot = img_l.clone();
  cv::Mat img_r_plot = img_r.clone();

  // compare
  for (int i = 0; i < matches_.size(); i++) {
    // 将l图象中的特征点像素转为l相机坐标系下坐标
    cv::Point2f point_camera =
        pixel2cam(keypoint_l_[matches_[i].queryIdx].pt, camera_matrix_);
    float depth = points_[i].z;
    cv::circle(img_l_plot, keypoint_l_[matches_[i].queryIdx].pt, 2,
               get_color(depth), 2);

    // 将同意特征点的3D坐标变换到r相机下
    cv::Mat point_camera_r =
        frame_rotation_ * (cv::Mat_<double>(3, 1) << points_[i].x, points_[i].y,
                           points_[i].z) +
        frame_translation_;

    float depth_r = point_camera_r.at<double>(2, 0);
    cv::circle(img_r_plot, keypoint_r_[matches_[i].queryIdx].pt, 2,
               get_color(depth_r), 2);

    cv::imwrite("triansgulation_l.jpg", img_l_plot);
    cv::imwrite("triansgulation_r.jpg", img_r_plot);
  }
}