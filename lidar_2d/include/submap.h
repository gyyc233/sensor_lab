#ifndef LIDAR_2D_SUB_MAP_H
#define LIDAR_2D_SUB_MAP_H
#ifdef ROS_CATKIN

#include "frame.h"
#include "likelihood_field.h"
#include "occupancy_map.h"

namespace sad {
/**
 * 子地图类
 * 子地图关联到若干个关键帧，也对应自己的一个栅格地图与似然场
 * 往子地图中添加关键帧时，会更新它的栅格地图与似然场
 * 子地图有自己的pose，记为
 * Tws，每个frame的世界位姿可以由子地图的pose乘frame在子地图中的相对pose得到
 */
class Submap {
public:
  Submap(const SE2 &pose) : pose_(pose) {
    occupancy_map_.setPose(pose);
    field_.setPose(pose);
  }

  /// @brief 把另一个submap中的占据栅格复制到本地图中
  /// @param other
  void setOccuFromOtherSubmap(std::shared_ptr<Submap> other);

  /// @brief 将frame与本submap进行匹配，计算frame->pose
  /// @param frame
  /// @return
  bool matchScan(std::shared_ptr<Frame> frame);

  /// @brief 判定当前的scan是否有位于submap外部的点
  /// @return
  bool hasOutsidePoints() const;

  /// @brief add a key frame for submap
  /// @param frame
  void addKeyFrame(std::shared_ptr<Frame> &frame) {
    frames_.emplace_back(frame);
  }

  /// @brief 在栅格地图中增加一个帧
  /// @param frame
  void addScanInOccupancyMap(std::shared_ptr<Frame> frame);

  /// @brief 当子地图的位姿更新时，重设每个frame的世界位姿
  void updateFramePoseWorld();

  OccupancyMap &getOccupancyMap() { return occupancy_map_; }

  LikelihoodField &getLikelihood() { return field_; }

  /// @brief get key frames
  /// @return
  std::vector<std::shared_ptr<Frame>> &getFrames() { return frames_; }

  size_t numFrames() const { return frames_.size(); }

  void setId(size_t id) { id_ = id; }

  size_t getId() const { return id_; }

  void setPose(const SE2 &pose);

  /// @brief 获取子地图的世界坐标
  /// @return
  SE2 getPose() const { return pose_; }

private:
  SE2 pose_; // submap pose based on world
  size_t id_ = 0;

  std::vector<std::shared_ptr<Frame>> frames_; // key frames of a submap
  LikelihoodField field_;                      // 用于匹配
  OccupancyMap occupancy_map_;                 // 用于生成栅格地图
};
} // namespace sad

#endif
#endif
