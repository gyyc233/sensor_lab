#ifdef ROS_CATKIN

#include "icp_2d.h"
#include "math_utils.h"
#include <glog/logging.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/search/impl/kdtree.hpp>

namespace sad {
void Icp2d::buildTargetKdTree() {
  if (target_scan_ == nullptr) {
    LOG(ERROR) << "target scan empty";
    return;
  }

  target_cloud_.reset(new Cloud2d);

  // Header header
  // float32 angle_min        # scan的开始角度 [弧度]
  // float32 angle_max        # scan的结束角度 [弧度]
  // float32 angle_increment  # 测量的角度间的距离 [弧度]
  // float32 time_increment   # 测量间的时间 [秒]
  // float32 scan_time        # 扫描间的时间 [秒]
  // float32 range_min        # 最小的测量距离 [米]
  // float32 range_max        # 最大的测量距离 [米]
  // float32[] ranges         # 测量的距离数据 [米] (注意: 值 < range_min 或 >
  // range_max 应当被丢弃) float32[] intensities    # 强度数据 [device-specific
  // units]

  for (size_t i = 0; i < target_scan_->ranges.size(); i++) {
    if (target_scan_->ranges[i] < target_scan_->range_min ||
        target_scan_->ranges[i] > target_scan_->range_max) {
      continue;
    }

    double real_angle =
        target_scan_->angle_min + i * target_scan_->angle_increment;

    Point2d p;
    p.x = target_scan_->ranges[i] * std::cos(real_angle);
    p.y = target_scan_->ranges[i] * std::sin(real_angle);
    target_cloud_->points.push_back(p);
  }

  target_cloud_->width = target_cloud_->points.size();
  target_cloud_->is_dense = false;
  kd_tree_.setInputCloud(target_cloud_);
}

bool Icp2d::alignGaussNewton(SE2 &init_pose) {
  int iterations = 10;
  double cost = 0, lastCost = 0;
  SE2 current_pose = init_pose;
  const float max_dis = 0.01;    // 最近邻时的最远距离（平方）
  const int min_effect_pts = 20; // 最小有效点数

  for (int iter = 0; iter < iterations; iter++) {
    Mat3d H = Mat3d::Zero();
    Vec3d b = Vec3d::Zero();
    cost = 0;

    int effective_num = 0; // 有效点数

    // 遍历source
    for (size_t i = 0; i < source_scan_->ranges.size(); ++i) {
      float r = source_scan_->ranges[i];
      if (r < source_scan_->range_min || r > source_scan_->range_max) {
        continue;
      }

      // lidar data to cartesian coordinate
      float angle = source_scan_->angle_min + i * source_scan_->angle_increment;
      float theta = current_pose.so2().log(); // SO2 对数映射到so2
      Vec2d pw = current_pose * Vec2d(r * std::cos(angle), r * std::sin(angle));
      Point2d pt;
      pt.x = pw.x();
      pt.y = pw.y();

      // 最近邻
      std::vector<int> nn_idx; // 最近点的索引
      std::vector<float> dis;  // 对应的距离
      // 在target中搜索距离pt最近的点
      kd_tree_.nearestKSearch(pt, 1, nn_idx, dis);

      // 存在最近点
      if (nn_idx.size() > 0 && dis[0] < max_dis) {
        effective_num++;
        Mat32d J;
        J << 1, 0, 0, 1, -r * std::sin(angle + theta),
            r * std::cos(angle + theta);
        H += J * J.transpose();

        Vec2d e(pt.x - target_cloud_->points[nn_idx[0]].x,
                pt.y - target_cloud_->points[nn_idx[0]].y);
        b += -J * e;
        cost += e.dot(e);
      }
    }

    if (effective_num < min_effect_pts) {
      return false;
    }

    // solve for dx
    Vec3d dx = H.ldlt().solve(b); // delta pose(x,y,theta)
    if (isnan(dx[0])) {
      break;
    }

    cost /= effective_num;
    if (iter > 0 && cost >= lastCost) {
      break;
    }

    LOG(INFO) << "iter " << iter << " cost = " << cost
              << ", effect num: " << effective_num;

    current_pose.translation() += dx.head<2>();
    current_pose.so2() =
        current_pose.so2() * SO2::exp(dx[2]); // theta to matrix
    lastCost = cost;
  }

  init_pose = current_pose;
  LOG(INFO) << "estimated pose: " << current_pose.translation().transpose()
            << ", theta: " << current_pose.so2().log();

  return true;
}

bool Icp2d::alignGaussNewtonPoint2Plane(SE2 &init_pose) {
  int iterations = 10;
  double cost = 0, lastCost = 0;
  SE2 current_pose = init_pose;
  const float max_dis = 0.3;     // 最近邻时的最远距离
  const int min_effect_pts = 20; // 最小有效点数

  for (int iter = 0; iter < iterations; iter++) {
    Mat3d H = Mat3d::Zero();
    Vec3d b = Vec3d::Zero();
    cost = 0;

    int effective_num = 0; // 有效点数

    // 遍历source
    for (size_t i = 0; i < source_scan_->ranges.size(); ++i) {
      float r = source_scan_->ranges[i];
      if (r < source_scan_->range_min || r > source_scan_->range_max) {
        continue;
      }

      float angle = source_scan_->angle_min + i * source_scan_->angle_increment;
      float theta = current_pose.so2().log();
      Vec2d pw = current_pose * Vec2d(r * std::cos(angle), r * std::sin(angle));
      Point2d pt;
      pt.x = pw.x();
      pt.y = pw.y();

      // 查找5个最近邻
      std::vector<int> nn_idx;
      std::vector<float> dis;
      kd_tree_.nearestKSearch(pt, 5, nn_idx, dis);

      std::vector<Vec2d> effective_pts; // 有效点
      for (int j = 0; j < nn_idx.size(); ++j) {
        if (dis[j] < max_dis) {
          effective_pts.emplace_back(Vec2d(target_cloud_->points[nn_idx[j]].x,
                                           target_cloud_->points[nn_idx[j]].y));
        }
      }

      if (effective_pts.size() < 3) {
        continue;
      }

      // 拟合直线，组装J、H和误差
      Vec3d line_coeffs;
      if (math::FitLine2D(effective_pts, line_coeffs)) {
        effective_num++;
        Vec3d J;
        J << line_coeffs[0], line_coeffs[1],
            -line_coeffs[0] * r * std::sin(angle + theta) +
                line_coeffs[1] * r * std::cos(angle + theta);
        H += J * J.transpose();

        double e =
            line_coeffs[0] * pw[0] + line_coeffs[1] * pw[1] + line_coeffs[2];
        b += -J * e;

        cost += e * e;
      }
    }

    if (effective_num < min_effect_pts) {
      return false;
    }

    // solve for dx
    Vec3d dx = H.ldlt().solve(b);
    if (isnan(dx[0])) {
      break;
    }

    cost /= effective_num;
    if (iter > 0 && cost >= lastCost) {
      break;
    }

    LOG(INFO) << "iter " << iter << " cost = " << cost
              << ", effect num: " << effective_num;

    current_pose.translation() += dx.head<2>();
    current_pose.so2() = current_pose.so2() * SO2::exp(dx[2]);
    lastCost = cost;
  }

  init_pose = current_pose;
  LOG(INFO) << "estimated pose: " << current_pose.translation().transpose()
            << ", theta: " << current_pose.so2().log();

  return true;
}

} // namespace sad

#endif
