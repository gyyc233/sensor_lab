#ifndef LIDAR_3D_IESKF_LIO_HPP
#define LIDAR_3D_IESKF_LIO_HPP
#ifdef ROS_CATKIN

#include <livox_ros_driver/CustomMsg.h>
#include <pcl/filters/voxel_grid.h>
#include <sensor_msgs/PointCloud2.h>

#include "iekf/iekf.hpp"
#include "incremental_ndt.h"
#include "loosely_coupled_lio/cloud_convert.h"
#include "loosely_coupled_lio/measure_sync.h"
#include "static_imu_init.h"

namespace sad {
class IEKF_LIO {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

  struct Options {
    Options() {}
    bool save_motion_undistortion_pcd_ = false; // 是否保存去畸变前后的点云
    bool with_ui_ = true;                       // 是否带着UI
  };

  IEKF_LIO(IEKF_LIO::Options options);
  ~IEKF_LIO() = default;

  /// init without ros
  bool Init(const std::string &config_yaml);

  /// 点云回调函数
  void PCLCallBack(const sensor_msgs::PointCloud2::ConstPtr &msg);
  void LivoxPCLCallBack(const livox_ros_driver::CustomMsg::ConstPtr &msg);

  /// IMU回调函数
  void IMUCallBack(IMUPtr msg_in);

  /// 结束程序，退出UI
  void Finish();

  /// 获取当前姿态
  NavStated GetCurrentState() const { return ieskf_.GetNominalState(); }

  /// 获取当前扫描
  CloudPtr GetCurrentScan() const { return current_scan_; }

private:
  bool LoadFromYAML(const std::string &yaml_file);

  /// 处理同步之后的IMU和雷达数据
  void ProcessMeasurements(const MeasureGroup &meas);

  /// 尝试让IMU初始化
  void TryInitIMU();

  /// 利用IMU预测状态信息
  /// 这段时间的预测数据会放入imu_states_里
  void Predict();

  /// 对measures_中的点云去畸变
  void Undistort();

  /// 执行一次配准和观测
  void Align();

  /// modules
  std::shared_ptr<MessageSync> sync_ = nullptr;
  StaticIMUInit imu_init_;

  /// point clouds data
  FullCloudPtr scan_undistort_{
      new FullPointCloudType()}; // scan after undistortion
  CloudPtr current_scan_ = nullptr;

  /// NDT数据
  IncrementalNdt3D ndt_;
  SE3 last_pose_;

  // flags
  bool imu_need_init_ = true;
  bool flg_first_scan_ = true;
  int frame_num_ = 0;

  ///////////////////////// EKF inputs and output
  //////////////////////////////////////////////////////////
  MeasureGroup measures_; // sync IMU and lidar scan
  std::vector<NavStated> imu_states_;
  IESKFD ieskf_; // IESKF
  SE3 TIL_;      // Lidar与IMU之间外参

  Options options_;
};
} // namespace sad

#endif
#endif
