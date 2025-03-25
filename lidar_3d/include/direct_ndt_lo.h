#ifndef LIDAR_3D_DIRECT_NDT_LO_H
#define LIDAR_3D_DIRECT_NDT_LO_H
#ifdef ROS_CATKIN

#include <deque>
#include <pcl/registration/ndt.h>

#include "eigen_type/eigen_types.h"
#include "ndt_3d.h"
#include "point_cloud/point_types.h"

namespace sad {

/// @brief 使用直接NDT方法进行递推的Lidar Odometry
/// @note 使用历史几个关键帧作为local map，进行NDT定位
class Direct_NDT_LO {
public:
  struct Options {
    Options() {}
    double kf_distance_ = 0.5;           // 关键帧距离
    double kf_angle_deg_ = 30;           // 旋转角度
    int num_kfs_in_local_map_ = 30;      // 局部地图含有多少个关键帧
    bool use_pcl_ndt_ = true;            // 使用本章的NDT还是PCL的NDT
    bool display_realtime_cloud_ = true; // 是否显示实时点云

    NDT_3D::Options ndt3d_options_; // NDT3D 的配置
  };

  Direct_NDT_LO() = delete;

  Direct_NDT_LO(Options options = Options());

  ~Direct_NDT_LO();

  /// @brief 往LO中增加一个点云
  /// @param scan 当前帧点云
  /// @param pose 估计pose
  void addCloud(CloudPtr scan, SE3 &pose);

  /// @brief 存储地图(viewer里）
  /// @param map_path
  void saveMap(const std::string &map_path);

private:
  /// @brief 与local map进行配准
  /// @param scan
  /// @return
  SE3 alignWithLocalMap(CloudPtr scan);

  /// @brief 判定是否为关键帧
  /// @param current_pose
  /// @return
  bool isKeyFrame(const SE3 &current_pose);

private:
  Options options_;
  CloudPtr local_map_ = nullptr;
  std::deque<CloudPtr> scans_in_local_map_; // 保存所有 key frame
  std::vector<SE3>
      estimated_poses_; // 所有估计出来的pose，用于记录轨迹和预测下一个帧
  SE3 last_kf_pose_; // 上一关键帧的位姿

  pcl::NormalDistributionsTransform<PointType, PointType> ndt_pcl_;
  NDT_3D ndt_;
};
} // namespace sad

#endif
#endif
