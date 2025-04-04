#ifdef ROS_CATKIN
#include "loosely_lio.h"
#include "navigation_and_mapping/lidar_utils.h"
#include <omp.h>
#include <pcl/common/transforms.h>
#include <yaml-cpp/yaml.h>

namespace sad {
LooselyLIO::LooselyLIO(Options options) : options_(options) {
  StaticIMUInit::Options imu_init_options;
  imu_init_options.use_speed_for_static_checking = false; // 不适用轮速计
  imu_init_ = StaticIMUInit(imu_init_options);
}

void LooselyLIO::PCLCallBack(const sensor_msgs::PointCloud2::ConstPtr &msg) {
  sync_->processCloud(msg);
}

void LooselyLIO::LivoxPCLCallBack(
    const livox_ros_driver::CustomMsg::ConstPtr &msg) {
  sync_->processCloud(msg);
}

void LooselyLIO::IMUCallBack(IMUPtr msg_in) { sync_->processIMU(msg_in); }

void LooselyLIO::finish() { LOG(INFO) << "finish done"; }

bool LooselyLIO::init(const std::string &config_yaml) {
  // 初始化自身参数
  if (!loadFromYAML(config_yaml)) {
    return false;
  }

  // 初始化NDT LO的参数
  sad::IncrementalNDTLO::Options in_ndt_options;
  in_ndt_options.display_realtime_cloud_ = false;
  inc_ndt_lo_ = std::make_shared<sad::IncrementalNDTLO>(in_ndt_options);

  return true;
}

bool LooselyLIO::loadFromYAML(const std::string &yaml_file) {
  // get params from yaml
  sync_ = std::make_shared<MessageSync>([this](const MeasureGroup &m) {
    // 处理同步之后的imu与雷达数据
    processMeasurements(m);
  });
  sync_->init(yaml_file);

  /// 自身参数主要是雷达与IMU外参
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

void LooselyLIO::processMeasurements(const MeasureGroup &meas) {
  LOG(INFO) << "call meas, imu: " << meas.imu_.size()
            << ", lidar pts: " << meas.lidar_->size();
  measures_ = meas;

  if (imu_need_init_) {
    // 初始化IMU系统
    tryInitIMU();
    return;
  }

  // 利用IMU数据进行状态预测
  predict();

  // 对点云去畸变
  undistort();

  // 配准
  align();
}

void LooselyLIO::predict() {
  imu_states_.clear();
  imu_states_.emplace_back(eskf_.GetNominalState());

  // 对imu状态进行预测
  for (auto &imu : measures_.imu_) {
    eskf_.Predict(*imu);
    imu_states_.emplace_back(eskf_.GetNominalState());
  }
}

void LooselyLIO::tryInitIMU() {
  for (auto imu : measures_.imu_) {
    imu_init_.AddIMU(*imu);
  }

  if (imu_init_.InitSuccess()) {
    // 读取初始零偏，设置ESKF
    sad::ESKFD::Options options;
    // 噪声由初始化器估计
    options.gyro_var = sqrt(imu_init_.GetCovGyro()[0]);
    options.acce_var = sqrt(imu_init_.GetCovAcce()[0]);
    eskf_.SetInitialConditions(options, imu_init_.GetInitBg(),
                               imu_init_.GetInitBa(), imu_init_.GetGravity());
    imu_need_init_ = false;

    LOG(INFO) << "IMU初始化成功";
  }
}

void LooselyLIO::undistort() {
  auto cloud = measures_.lidar_;
  auto imu_state = eskf_.GetNominalState(); // 最后时刻的状态
  SE3 T_end = SE3(imu_state.R_, imu_state.p_);

  if (options_.save_motion_undistortion_pcd_) {
    pcl::io::savePCDFileASCII("./data/mapping_3d/loosely_lio_before_undist.pcd",
                              *cloud);
  }

/// 将所有点转到最后时刻状态上
#pragma omp parallel
  {
    std::for_each(cloud->points.begin(), cloud->points.end(), [&](auto &pt) {
      SE3 Ti = T_end;
      NavStated match;

      // 根据pt.time查找时间，pt.time是该点打到的时间与雷达开始时间之差，单位为毫秒
      math::PoseInterp<NavStated>(
          measures_.lidar_begin_time_ + pt.time * 1e-3, imu_states_,
          [](const NavStated &s) { return s.timestamp_; },
          [](const NavStated &s) { return s.GetSE3(); }, Ti, match);

      Vec3d pi = ToVec3d(pt);
      // T_i scans起始时间imu位姿
      // T_end 结束时刻imu位姿
      Vec3d p_compensate = TIL_.inverse() * T_end.inverse() * Ti * TIL_ * pi;

      pt.x = p_compensate(0);
      pt.y = p_compensate(1);
      pt.z = p_compensate(2);
    });
  }

  scan_undistort_ = cloud;

  if (options_.save_motion_undistortion_pcd_) {
    pcl::io::savePCDFileASCII("./data/mapping_3d/loosely_lio_after_undist.pcd",
                              *cloud);
  }
}

void LooselyLIO::align() {
  FullCloudPtr scan_undistort_trans(new FullPointCloudType);
  pcl::transformPointCloud(*scan_undistort_, *scan_undistort_trans,
                           TIL_.matrix());
  scan_undistort_ = scan_undistort_trans;

  auto current_scan = ConvertToCloud<FullPointType>(scan_undistort_);

  // voxel
  pcl::VoxelGrid<PointType> voxel;
  voxel.setLeafSize(0.5, 0.5, 0.5);
  voxel.setInputCloud(current_scan);

  CloudPtr current_scan_filter(new PointCloudType);
  voxel.filter(*current_scan_filter);

  /// 处理首帧雷达数据
  if (flg_first_scan_) {
    SE3 pose;
    inc_ndt_lo_->addCloud(current_scan_filter, pose);
    flg_first_scan_ = false;
    return;
  }

  /// 从EKF中获取预测pose，放入LO，获取LO位姿，最后合入EKF
  SE3 pose_predict = eskf_.GetNominalSE3();
  inc_ndt_lo_->addCloud(current_scan_filter, pose_predict, true);
  pose_of_lo_ = pose_predict;
  eskf_.ObserveSE3(pose_of_lo_, 1e-2, 1e-2);
  frame_num_++;
}

} // namespace sad

#endif
