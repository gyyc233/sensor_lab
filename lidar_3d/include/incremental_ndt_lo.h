#ifndef LIDAR_3D_INCREMENTAL_NDT_LO_H
#define LIDAR_3D_INCREMENTAL_NDT_LO_H
#ifdef ROS_CATKIN

#include "eigen_type/eigen_types.h"
#include "incremental_ndt.h"
#include "point_cloud/pcl_map_viewer.h"
#include "point_cloud/point_types.h"

namespace sad {

/**
 * 使用直接NDT方法进行递推的Lidar Odometry
 * 使用历史几个关键帧作为local map，进行NDT定位
 */
class IncrementalNDTLO {
public:
  struct Options {
    Options() {}
    double kf_distance_ = 0.5;                // 关键帧距离
    double kf_angle_deg_ = 30;                // 旋转角度
    bool display_realtime_cloud_ = true;      // 是否显示实时点云
    IncrementalNdt3D::Options ndt3d_options_; // NDT3D 的配置
  };

  IncrementalNDTLO(Options options = Options()) : options_(options) {
    if (options_.display_realtime_cloud_) {
      viewer_ = std::make_shared<PCLMapViewer>(0.5);
    }
    ndt_ = IncrementalNdt3D(options_.ndt3d_options_);
  }

  /**
   * 往LO中增加一个点云
   * @param scan  当前帧点云
   * @param pose 估计pose
   */
  void addCloud(CloudPtr scan, SE3 &pose, bool use_guess = false);

  /// 存储地图(viewer里）
  void saveMap(const std::string &map_path);

private:
  /// 判定是否为关键帧
  bool isKeyframe(const SE3 &current_pose);

private:
  Options options_;
  bool first_frame_ = true;
  std::vector<SE3>
      estimated_poses_; // 所有估计出来的pose，用于记录轨迹和预测下一个帧
  SE3 last_kf_pose_; // 上一关键帧的位姿
  int cnt_frame_ = 0;

  IncrementalNdt3D ndt_;
  std::shared_ptr<PCLMapViewer> viewer_ = nullptr;
};

} // namespace sad

#endif
#endif // SLAM_IN_AUTO_DRIVING_INCREMENTAL_NDT_LO_H
