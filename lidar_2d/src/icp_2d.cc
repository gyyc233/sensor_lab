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

  for (size_t i = 0; i < target_scan_.size(); i++) {
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
  kdt_ree_.setInputCloud(target_cloud_);
}
} // namespace sad

#endif
