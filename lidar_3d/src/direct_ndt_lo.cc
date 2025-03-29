#ifdef ROS_CATKIN

#include "direct_ndt_lo.h"
#include "math_utils.h"

#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>

namespace sad {

Direct_NDT_LO::Direct_NDT_LO(Options options) : options_(options) {
  ndt_ = NDT_3D(options_.ndt3d_options_);
  ndt_pcl_.setResolution(1.0);
  ndt_pcl_.setStepSize(0.10);
  ndt_pcl_.setTransformationEpsilon(0.1);
}

Direct_NDT_LO::~Direct_NDT_LO() {}

void Direct_NDT_LO::addCloud(CloudPtr scan, SE3 &pose) {
  // 第一帧直接加入local map
  if (local_map_ == nullptr) {
    local_map_.reset(new PointCloudType);
    // operator += 用来拼接点云
    *local_map_ += *scan;
    pose = SE3();
    last_kf_pose_ = pose;

    if (options_.use_pcl_ndt_) {
      ndt_pcl_.setInputTarget(local_map_);
    } else {
      ndt_.setTarget(local_map_);
    }
    return;
  }

  // 不是第一帧
  // 计算scan相对于local map的位姿
  pose = alignWithLocalMap(scan);
  CloudPtr scan_world(new PointCloudType);
  // scan 右乘 pose 得到scan_world
  pcl::transformPointCloud(*scan, *scan_world, pose.matrix().cast<float>());

  // 根据位姿变化判断是否为关键帧
  if (isKeyFrame(pose)) {
    last_kf_pose_ = pose;

    // 重建local map
    scans_in_local_map_.emplace_back(scan_world);
    if (scans_in_local_map_.size() > options_.num_kfs_in_local_map_) {
      scans_in_local_map_.pop_front();
    }

    local_map_.reset(new PointCloudType);
    for (auto &scan : scans_in_local_map_) {
      *local_map_ += *scan;
    }

    if (options_.use_pcl_ndt_) {
      ndt_pcl_.setInputTarget(local_map_);
    } else {
      ndt_.setTarget(local_map_);
    }
  }

  // TODO: visualize
}

bool Direct_NDT_LO::isKeyFrame(const SE3 &current_pose) {
  // 只要与上一帧相对运动超过一定距离或角度，就记关键帧
  // 假设上一帧右乘delta等于current_pose
  SE3 delta = last_kf_pose_.inverse() * current_pose;
  return delta.translation().norm() > options_.kf_distance_ ||
         delta.so3().log().norm() > options_.kf_angle_deg_ * math::kDEG2RAD;
}

void Direct_NDT_LO::saveMap(const std::string &map_path) {
  if (local_map_->points.size() > 0) {
    pcl::io::savePCDFileASCII(map_path, *local_map_);
    LOG(INFO) << "save map to: " << map_path;
  } else {
    LOG(INFO) << "map was empty";
  }
}

SE3 Direct_NDT_LO::alignWithLocalMap(CloudPtr scan) {
  if (options_.use_pcl_ndt_) {
    ndt_pcl_.setInputSource(scan);
  } else {
    ndt_.setSource(scan);
  }

  CloudPtr output(new PointCloudType());

  SE3 guess;
  bool align_success = true;
  if (estimated_poses_.size() < 2) {
    if (options_.use_pcl_ndt_) {
      ndt_pcl_.align(*output, guess.matrix().cast<float>());
      guess =
          Mat4ToSE3(ndt_pcl_.getFinalTransformation().cast<double>().eval());
    } else {
      align_success = ndt_.alignNewtonGaussianNdt(guess);
    }
  } else {
    // 从最近两个pose来推断初始值　emms
    SE3 T1 = estimated_poses_[estimated_poses_.size() - 1];
    SE3 T2 = estimated_poses_[estimated_poses_.size() - 2];
    guess = T1 * (T2.inverse() * T1);

    if (options_.use_pcl_ndt_) {
      ndt_pcl_.align(*output, guess.matrix().cast<float>());
      guess =
          Mat4ToSE3(ndt_pcl_.getFinalTransformation().cast<double>().eval());
    } else {
      align_success = ndt_.alignNewtonGaussianNdt(guess);
    }
  }

  LOG(INFO) << "pose: " << guess.translation().transpose() << ", "
            << guess.so3().unit_quaternion().coeffs().transpose();

  if (options_.use_pcl_ndt_) {
    LOG(INFO) << "trans prob: " << ndt_pcl_.getTransformationProbability();
  }

  estimated_poses_.emplace_back(guess);
  return guess;
}

} // namespace sad

#endif
