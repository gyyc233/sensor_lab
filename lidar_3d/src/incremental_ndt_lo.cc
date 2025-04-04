#ifdef ROS_CATKIN
#include "incremental_ndt_lo.h"
#include "math_utils.h"
#include <pcl/common/transforms.h>

namespace sad {

void IncrementalNDTLO::addCloud(CloudPtr scan, SE3 &pose, bool use_guess) {
  if (first_frame_) {
    // 第一个帧，直接加入local map
    pose = SE3();
    last_kf_pose_ = pose;
    ndt_.addCloud(scan);
    first_frame_ = false;
    return;
  }

  // 此时local map位于NDT内部，直接配准即可
  SE3 guess;
  ndt_.setSource(scan);
  if (estimated_poses_.size() < 2) {
    ndt_.alignNdt(guess);
  } else {
    if (!use_guess) {
      // 从最近两个pose来推断
      SE3 T1 = estimated_poses_[estimated_poses_.size() - 1];
      SE3 T2 = estimated_poses_[estimated_poses_.size() - 2];
      guess = T1 * (T2.inverse() * T1);
    } else {
      guess = pose;
    }

    ndt_.alignNdt(guess);
  }

  pose = guess;
  estimated_poses_.emplace_back(pose);

  CloudPtr scan_world(new PointCloudType);
  pcl::transformPointCloud(*scan, *scan_world, guess.matrix().cast<float>());

  if (isKeyframe(pose)) {
    last_kf_pose_ = pose;
    cnt_frame_ = 0;
    // 放入ndt内部的local map
    ndt_.addCloud(scan_world);
  }

  cnt_frame_++;
}

bool IncrementalNDTLO::isKeyframe(const SE3 &current_pose) {
  if (cnt_frame_ > 10) {
    return true;
  }

  SE3 delta = last_kf_pose_.inverse() * current_pose;
  return delta.translation().norm() > options_.kf_distance_ ||
         delta.so3().log().norm() > options_.kf_angle_deg_ * math::kDEG2RAD;
}

void IncrementalNDTLO::saveMap(const std::string &map_path) {
  // TODO:
}

} // namespace sad

#endif
