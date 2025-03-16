#ifdef ROS_CATKIN

#include "submap.h"
#include <glog/logging.h>

namespace sad {
void Submap::setOccuFromOtherSubmap(std::shared_ptr<Submap> other) {
  auto frames_in_other = other->getFrames();

  for (size_t i = 0; i < frames_in_other.size(); i++) {
    if (i > 9) {
      break;
    }
    occupancy_map_.addLidarFrame(frames_in_other[i]);
  }

  // 根据当前栅格生成似然场,以提供给当前子地图
  field_.setFieldImageFromOccuMap(occupancy_map_.getOccupancyGrid());
}

bool Submap::matchScan(std::shared_ptr<Frame> frame) {
  // 在似然场中设置扫描数据为被配准对象
  field_.setSourceScan(frame->scan_);
  // 计算scan相对于submap的位姿
  field_.alignG2O(frame->pose_submap_); // 将优化后的位姿保存在pose_submap_

  // 将scan位姿转到世界坐标系下
  frame->pose_ = pose_ * frame->pose_submap_;

  return true;
}

bool Submap::hasOutsidePoints() const {
  return occupancy_map_.hasOutsidePoints();
}

void Submap::addScanInOccupancyMap(std::shared_ptr<Frame> frame) {
  occupancy_map_.addLidarFrame(frame);
  // 更新似然场
  field_.setFieldImageFromOccuMap(occupancy_map_.getOccupancyGrid());
}

void Submap::updateFramePoseWorld() {
  std::for_each(frames_.begin(), frames_.end(), [this](const auto &frame) {
    // 重新计算关键帧在世界坐标系下的位姿
    frame->pose_ = pose_ * frame->pose_submap_;
  });
}

} // namespace sad

#endif
