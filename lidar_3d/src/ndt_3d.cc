#ifdef ROS_CATKIN
#include "ndt_3d.h"
#include "math_utils.h"
#include "navigation_and_mapping/lidar_utils.h"

#include <Eigen/SVD>
#include <glog/logging.h>

namespace sad {

NDT_3D::NDT_3D() {
  options_.inv_voxel_size_ = 1.0 / options_.inv_voxel_size_;
  generateNearbyGrids();
}

NDT_3D(Options options) : options_(options) {
  options_.inv_voxel_size_ = 1.0 / options_.inv_voxel_size_;
  generateNearbyGrids();
}

NDT_3D::~NDT_3D() {}

void NDT_3D::generateNearbyGrids() {
  if (options_.nearby_type_ == NearbyTYpe::CENTER) {
    nearby_grids_.emplace_back(KeyType::Zero());
  } else if (options_.nearby_type_ == NearbyTYpe::NEARBY6) {
    nearby_grids_ = {KeyType(0, 0, 0), KeyType(-1, 0, 0), KeyType(1, 0, 0),
                     KeyType(0, 1, 0), KeyType(0, -1, 0), KeyType(0, 0, -1),
                     KeyType(0, 0, 1)};
  }
}

void NDT_3D::setTarget(CloudPtr target) {
  target_ = target;
  buildVoxels();

  // 计算点云中心
  target_center_ =
      std::accumulate(target->points.begin(), target_->points.end(),
                      Vec3d::Zero().eval(),
                      [](const Vec3d &c, const PointType &pt) -> Vec3d {
                        return c + ToVec3d(pt);
                      }) /
      target_->size();
}

void NDT_3D::setSource(CloudPtr source) {
  source_ = source;

  source_center_ =
      std::accumulate(source_->points.begin(), source_->points.end(),
                      Vec3d::Zero().eval(),
                      [](const Vec3d &c, const PointType &pt) -> Vec3d {
                        return c + ToVec3d(pt);
                      }) /
      source_->size();
}

void NDT_3D::setGtPose(const SE3 &gt_pose) {
  gt_pose_ = gt_pose;
  gt_set_ = true;
}

void NDT_3D::buildVoxels() {
  assert(target_ != nullptr);
  assert(target_->empty() == false);
  grids_.clear();

  /// 分配体素
  std::vector<size_t> index(target_->size());
  int id = 0;
  std::for_each(index.begin(), index.end(),
                [&id](size_t &i) mutable { i = id++; });

  std::for_each(index.begin(), index.end(), [this](const size_t &idx) {
    // TODO: 这里inv_voxel_size_的作用没明白
    Vec3d pt = ToVec3d(target_->points[idx]) * options_.inv_voxel_size_;
    auto key = CastToInt(pt);
    if (grids_.find(key) == grids_.end()) {
      grids_.insert({key, {idx}});
    } else {
      // 若其它点处于同一个体素中，则加入idx
      grids_[key].idx_.emplace_back(idx);
    }
  });

  /// 删除点数不够的
  for (auto iter = grids_.begin(); iter != grids_.end();) {
    if (iter->second.idx_.size() > options_.min_pts_in_voxel_) {
      iter++;
    } else {
      iter = grids_.erase(iter);
    }
  }

  /// 计算每个体素中的均值和协方差
  std::for_each(grids_.begin(), grids_.end(), [this](auto &v) {
    // 该体素内超过３个点
    if (v.second.idx_.size() > options_.min_pts_in_voxel_) {
      math::ComputeMeanAndCov(
          v.second.idx_, v.second.mu_, v.second.sigma_,
          [this](const size_t &idx) { return ToVec3d(target_->points[idx]); });
      // SVD 检查最大与最小奇异值，限制最小奇异值

      Eigen::JacobiSVD svd(v.second.sigma_,
                           Eigen::ComputeFullU | Eigen::ComputeFullV);
      Vec3d lambda = svd.singularValues();
      if (lambda[1] < lambda[0] * 1e-3) {
        lambda[1] = lambda[0] * 1e-3;
      }

      if (lambda[2] < lambda[0] * 1e-3) {
        lambda[2] = lambda[0] * 1e-3;
      }

      // asDiagonal() 将数组转为对角阵
      Mat3d inv_lambda =
          Vec3d(1.0 / lambda[0], 1.0 / lambda[1], 1.0 / lambda[2]).asDiagonal();

      // v.second.info_ = (v.second.sigma_ + Mat3d::Identity() *
      // 1e-3).inverse();  // 避免出nan

      // 计算协方差矩阵的逆
      v.second.info_ = svd.matrixV() * inv_lambda * svd.matrixU().transpose();
    }
  });
}

bool NDT_3D::alignNewtonGaussianNdt(SE3 &init_pose) {
  LOG(INFO) << "align with NDT and newton gaussian";
  assert(grids_.empty() == false);

  Sophus::SE3 pose = init_pose;
  if (options_.remove_centroid_) {
    pose.translation() = target_center_ - source_center_;
    LOG(INFO) << "init trans set to " << pose.translation().transpose();
  }

  // 对点的索引，预先生成
  int num_residual_per_point = 1;
  if (options_.nearby_type_ == NearbyType::NEARBY6) {
    num_residual_per_point = 7;
  }

  std::vector<int> index(source_->points.size());
  for (int i = 0; i < index.size(); ++i) {
    index[i] = i;
  }

  int total_size = index.size() * num_residual_per_point;

  for (int iter = 0; iter < options_.max_iteration_; ++iter) {
    std::vector<bool> effect_pts(total_size, false);
    std::vector<Eigen::Matrix<double, 3, 6>> jacobians(total_size);
    std::vector<Vec3d> errors(total_size);
    std::vector<Mat3d> infos(total_size);

    // gauss-newton
    std::for_each(index.begin(), index.end(), [&](int idx) {
      auto q = ToVec3d(source_->points[idx]);
      // source点q转到target附近，寻找他对应的体素id
      Vec3d qs = pose * q;

      // 计算qs所在的栅格以及它的最近邻栅格
      Vec3i key = CastToInt(Vec3d(qs * options_.inv_voxel_size_));

      for (int i = 0; i < nearby_grids_.size(); ++i) {
        auto key_off = key + nearby_grids_[i]; // 获取同一体素内的点
        auto it = grids_.find(key_off);
        int real_idx = idx * num_residual_per_point + i;
        if (it != grids_.end()) {

          auto &v = it->second; // voxel
          // 使用体素均值计算残差
          Vec3d e = qs - v.mu_;

          // check chi2 th
          double res = e.transpose() * v.info_ * e;
          if (std::isnan(res) || res > options_.res_outlier_th_) {
            effect_pts[real_idx] = false;
            continue;
          }

          // build residual
          Eigen::Matrix<double, 3, 6> J;
          J.block<3, 3>(0, 0) = -pose.so3().matrix() * SO3::hat(q);
          J.block<3, 3>(0, 3) = Mat3d::Identity();

          jacobians[real_idx] = J;
          errors[real_idx] = e;
          infos[real_idx] = v.info_;
          effect_pts[real_idx] = true;
        } else {
          effect_pts[real_idx] = false;
        }
      }
    });

    // 累加Hessian和error,计算dx
    double total_res = 0;
    int effective_num = 0;

    Mat6d H = Mat6d::Zero();
    Vec6d err = Vec6d::Zero();

    for (int idx = 0; idx < effect_pts.size(); ++idx) {
      if (!effect_pts[idx]) {
        continue;
      }

      total_res += errors[idx].transpose() * infos[idx] * errors[idx];
      // chi2.emplace_back(errors[idx].transpose() * infos[idx] * errors[idx]);
      effective_num++;

      // 这里使用了信息矩阵
      H += jacobians[idx].transpose() * infos[idx] * jacobians[idx];
      err += -jacobians[idx].transpose() * infos[idx] * errors[idx];
    }

    if (effective_num < options_.min_effective_pts_) {
      LOG(WARNING) << "effective num too small: " << effective_num;
      return false;
    }

    Vec6d dx = H.inverse() * err;

    // 右乘扰动更新项
    pose.so3() = pose.so3() * SO3::exp(dx.head<3>());
    pose.translation() += dx.tail<3>();

    // 更新
    LOG(INFO) << "iter " << iter << " total res: " << total_res
              << ", eff: " << effective_num
              << ", mean res: " << total_res / effective_num
              << ", dxn: " << dx.norm() << ", dx: " << dx.transpose();

    if (gt_set_) {
      double pose_error = (gt_pose_.inverse() * pose).log().norm();
      LOG(INFO) << "iter " << iter << " pose error: " << pose_error;
    }

    if (dx.norm() < options_.eps_) {
      LOG(INFO) << "converged, dx = " << dx.transpose();
      break;
    }
  }

  // 优化后的初值
  init_pose = pose;
  return true;
}

} // namespace sad

#endif
