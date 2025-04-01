#ifndef TOOLS_VELODYNE_CONVERT_H
#define TOOLS_VELODYNE_CONVERT_H
#ifdef ROS_CATKIN

#include "navigation_and_mapping/lidar_utils.h"
#include "point_cloud/point_types.h"
#include "tools/pointcloud_convert/packets_parser.h"

namespace sad::tools {

/// velodyne的配置参数
struct VelodyneConfig {
  int type = 16;
  double xoffset = 0.333;
  double yoffset = 0;
  double zoffset = 1.17;
  double roll = -0.085;
  double pitch = 0;
  double yaw = 90.7557;

  double max_range = 60.0;
  double min_range = 1.0;
  double max_angle = 0;
  double min_angle = 0;
  bool is_organized = false;
  double view_direction = 0;
  double view_width = 0;
  bool enable_coordinate_transformation = true;
  double car_left = 1.0;
  double car_right = -1.0;
  double car_front = 2.0;
  double car_back = -2.0;
  double car_top = 2.0;
  double car_bottom = -20.0;
};

/// velodyne输出的packets转换成pointcloud格式
/// 实质上只是对packets_parser外面再包了一层
class VelodyneConvertor {
public:
  explicit VelodyneConvertor(const VelodyneConfig &config = VelodyneConfig());

  /**
   * 将packets msgs转换为FullCloud
   * 同时会根据velodyne_config_中的配置来将激光点云转到IMU系
   * @param packets_msg
   * @param out_cloud
   */
  void ProcessScan(const PacketsMsgPtr &packets_msg, FullCloudPtr &out_cloud);

private:
  VelodyneConfig velodyne_config_;
  std::shared_ptr<PacketsParser> packets_parser_ = nullptr;
  FullCloudPtr converted_cloud_ = nullptr;
};
} // namespace sad::tools

#endif
#endif
