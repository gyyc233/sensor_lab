#ifdef ROS_CATKIN
#include "iekf_lio.h"
#include "navigation_and_mapping/lidar_utils.h"
#include <fstream>
#include <pcl/common/transforms.h>
#include <yaml-cpp/yaml.h>

namespace sad {
IEKF_LIO::IEKF_LIO(IEKF_LIO::Options options) : options_(options) {
  StaticIMUInit::Options imu_init_options;
  imu_init_options.use_speed_for_static_checking =
      false; // 本节数据不需要轮速计
  imu_init_ = StaticIMUInit(imu_init_options);
}

bool IEKF_LIO::Init(const std::string &config_yaml) {
  if (!LoadFromYAML(config_yaml)) {
    LOG(INFO) << "init failed.";
    return false;
  }

  return true;
}

bool IEKF_LIO::LoadFromYAML(const std::string &yaml_file) {
  // get params from yaml
  sync_ = std::make_shared<MessageSync>(
      [this](const MeasureGroup &m) { ProcessMeasurements(m); });

  // 雷达 imu 外参
  auto yaml = YAML::LoadFile(yaml_file);
  std::vector<double> ext_t =
      yaml["mapping"]["extrinsic_T"].as<std::vector<double>>();
  std::vector<double> ext_r =
      yaml["mapping"]["extrinsic_R"].as<std::vector<double>>();

  Vec3d lidar_T_wrt_IMU = math::VecFromArray(ext_t);
  Mat3d lidar_R_wrt_IMU = math::MatFromArray(ext_r);
  TIL_ = SE3(lidar_R_wrt_IMU, lidar_T_wrt_IMU);
  return true;
}

void IEKF_LIO::ProcessMeasurements(const MeasureGroup &meas) {
  LOG(INFO) << "call meas, imu: " << meas.imu_.size()
            << ", lidar pts: " << meas.lidar_->size();
  measures_ = meas;

  if (imu_need_init_) {
    // 初始化IMU系统
    TryInitIMU();
    return;
  }

  // 利用IMU数据进行状态预测
  Predict();

  // 对点云去畸变
  Undistort();

  // 配准
  Align();
}

void IEKF_LIO::Align() {
  FullCloudPtr scan_undistort_trans(new FullPointCloudType);
  // scan转到imu
  pcl::transformPointCloud(*scan_undistort_, *scan_undistort_trans,
                           TIL_.matrix().cast<float>());
  scan_undistort_ = scan_undistort_trans;
  current_scan_ = ConvertToCloud<FullPointType>(scan_undistort_);

  pcl::VoxelGrid<PointType> voxel;
  voxel.setLeafSize(0.5, 0.5, 0.5);
  voxel.setInputCloud(current_scan_);

  CloudPtr current_scan_filter(new PointCloudType);
  voxel.filter(*current_scan_filter);

  /// the first scan
  if (flg_first_scan_) {
    ndt_.addCloud(current_scan_);
    flg_first_scan_ = false;

    return;
  }

  // 后续的scan，使用NDT配合pose进行更新
  LOG(INFO) << "=== frame " << frame_num_;

  ndt_.setSource(current_scan_filter);
  ieskf_.UpdateUsingCustomObserve(
      [this](const SE3 &input_pose, Mat18d &HTVH, Vec18d &HTVr) {
        ndt_.computeResidualAndJacobians(input_pose, HTVH, HTVr);
      });

  auto current_nav_state = ieskf_.GetNominalState();

  // 若运动了一定范围，则把点云放入地图中
  SE3 current_pose = ieskf_.GetNominalSE3();
  SE3 delta_pose = last_pose_.inverse() * current_pose;

  if (delta_pose.translation().norm() > 1.0 ||
      delta_pose.so3().log().norm() > math::deg2rad(10.0)) {
    // 将地图合入NDT中
    CloudPtr current_scan_world(new PointCloudType);
    pcl::transformPointCloud(*current_scan_filter, *current_scan_world,
                             current_pose.matrix());
    ndt_.addCloud(current_scan_world);
    last_pose_ = current_pose;
  }

  frame_num_++;
  return;
}

void IEKF_LIO::TryInitIMU() {
  for (auto imu : measures_.imu_) {
    imu_init_.AddIMU(*imu);
  }

  if (imu_init_.InitSuccess()) {
    // 读取初始零偏，设置ESKF
    sad::IESKFD::Options options;
    // 噪声由初始化器估计
    options.gyro_var_ = sqrt(imu_init_.GetCovGyro()[0]);
    options.acce_var_ = sqrt(imu_init_.GetCovAcce()[0]);
    ieskf_.SetInitialConditions(options, imu_init_.GetInitBg(),
                                imu_init_.GetInitBa(), imu_init_.GetGravity());
    imu_need_init_ = false;

    LOG(INFO) << "IMU初始化成功";
  }
}

void IEKF_LIO::Undistort() {
  auto cloud = measures_.lidar_;
  auto imu_state = ieskf_.GetNominalState(); // 最后时刻的状态
  SE3 T_end = SE3(imu_state.R_, imu_state.p_);

  if (options_.save_motion_undistortion_pcd_) {
    pcl::io::savePCDFileASCII("./data/mapping_3d/iekf_lio_before_undist.pcd",
                              *cloud);
  }

  /// 将所有点转到最后时刻状态上
  std::for_each(cloud->points.begin(), cloud->points.end(), [&](auto &pt) {
    SE3 Ti = T_end;
    NavStated match;

    // 根据pt.time查找时间，pt.time是该点打到的时间与雷达开始时间之差，单位为毫秒
    math::PoseInterp<NavStated>(
        measures_.lidar_begin_time_ + pt.time * 1e-3, imu_states_,
        [](const NavStated &s) { return s.timestamp_; },
        [](const NavStated &s) { return s.GetSE3(); }, Ti, match);

    Vec3d pi = ToVec3d(pt);
    Vec3d p_compensate = TIL_.inverse() * T_end.inverse() * Ti * TIL_ * pi;

    pt.x = p_compensate(0);
    pt.y = p_compensate(1);
    pt.z = p_compensate(2);
  });
  scan_undistort_ = cloud;

  if (options_.save_motion_undistortion_pcd_) {
    pcl::io::savePCDFileASCII("./data/mapping_3d/iekf_lio_after_undist.pcd",
                              *cloud);
  }
}

void IEKF_LIO::Predict() {
  imu_states_.clear();
  imu_states_.emplace_back(ieskf_.GetNominalState());

  /// 对IMU状态进行预测
  for (auto &imu : measures_.imu_) {
    ieskf_.Predict(*imu);
    imu_states_.emplace_back(ieskf_.GetNominalState());
  }
}

void IEKF_LIO::PCLCallBack(const sensor_msgs::PointCloud2::ConstPtr &msg) {
  sync_->processCloud(msg);
}

void IEKF_LIO::LivoxPCLCallBack(
    const livox_ros_driver::CustomMsg::ConstPtr &msg) {
  sync_->processCloud(msg);
}

void IEKF_LIO::IMUCallBack(IMUPtr msg_in) { sync_->processIMU(msg_in); }

void IEKF_LIO::Finish() { LOG(INFO) << "finish done"; }

} // namespace sad

#endif
