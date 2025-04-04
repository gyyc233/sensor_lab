#ifdef ROS_CATKIN

#include "icp_3d.h"
#include "math_utils.h"
#include <omp.h>

namespace sad {

ICP_3D::ICP_3D() {}

ICP_3D::ICP_3D(Options optional) : options_(optional) {}

ICP_3D::~ICP_3D() {}

void ICP_3D::buildTargetKdTree() {
  kd_tree_ = std::make_shared<KdTree>();
  kd_tree_->BuildTree(target_);
  // 计算近似最邻近
  kd_tree_->SetEnableANN();
}

void ICP_3D::setTarget(pcl::PointCloud<pcl::PointXYZI>::Ptr target) {
  target_ = target;
  buildTargetKdTree();

  target_center_ =
      std::accumulate(target_->points.begin(), target_->points.end(),
                      Eigen::Vector3d::Zero().eval(),
                      [](const Eigen::Vector3d &c,
                         const pcl::PointXYZI &pt) -> Eigen::Vector3d {
                        return c + pt.getVector3fMap().cast<double>();
                      }) /
      target_->size();

  LOG(INFO) << "target center: " << target_center_.transpose();
}

void ICP_3D::setSource(pcl::PointCloud<pcl::PointXYZI>::Ptr source) {
  source_ = source;
  source_center_ =
      std::accumulate(source_->points.begin(), source_->points.end(),
                      Vec3d::Zero().eval(),
                      [](const Vec3d &c, const PointType &pt) -> Vec3d {
                        return c + ToVec3d(pt);
                      }) /
      source_->size();
  LOG(INFO) << "source center: " << source_center_.transpose();
}

void ICP_3D::setGroundTruth(const SE3 &gt_pose) {
  gt_pose_ = gt_pose;
  gt_set_ = true;
}

bool ICP_3D::AlignP2P(SE3 &init_pose) {
  LOG(INFO) << "aligning with point to point";
  assert(target_ != nullptr && source_ != nullptr);

  SE3 pose = init_pose;
  if (!options_.use_initial_translation_) {
    pose.translation() = target_center_ - source_center_; // 设置平移初始值
  }
  LOG(INFO) << "init guess pose: " << pose;

  // 对点的索引，预先生成
  std::vector<int> index(source_->points.size());
  for (int i = 0; i < index.size(); ++i) {
    index[i] = i;
  }

  std::vector<bool> effect_pts(index.size(), false);
  std::vector<Eigen::Matrix<double, 3, 6>> jacobians(index.size());
  std::vector<Vec3d> errors(index.size());

  for (int iter = 0; iter < options_.max_iteration_; ++iter) {
// gauss-newton
#pragma omp parallel
    {
      std::for_each(index.begin(), index.end(), [&](int idx) {
        auto q = source_->points[idx].getVector3fMap().cast<double>();
        Vec3d qs = pose * q; // 使用初始位姿进行转换
        // knn
        std::vector<int> nn;
        kd_tree_->GetClosestPoint(ToPointType(qs), nn, 1);

        if (!nn.empty()) {
          // target中的p是 qs(source)的最邻近点
          Vec3d p = ToVec3d(target_->points[nn[0]]);
          double dis2 = (p - qs).squaredNorm();
          if (dis2 > options_.max_nn_distance_) {
            effect_pts[idx] = false;
            return; // std::for_each
                    // 初衷就是把这段区间都执行完，否则你就用for代替for_each
          }
          effect_pts[idx] = true;

          // build residual
          Vec3d e = p - qs;
          // 因为假设是source经过rt变换到target 问题描述为
          // e = |p_t - (R*p_s + T)|取最小
          // e分别对位姿的旋转和平移部分求偏导 则jacobian为 3*6 使用右乘扰动模型
          Eigen::Matrix<double, 3, 6> J;
          J.block<3, 3>(0, 0) =
              -pose.so3().matrix() *
              SO3::hat(q); // 对旋转求导，右乘扰动项旋转部分导数 -R(q^)
          J.block<3, 3>(0, 3) =
              Mat3d::Identity(); // 对平移求导，右乘扰动项平移部分导数 I
          J = -1 * J;

          jacobians[idx] = J;
          errors[idx] = e;
        } else {
          effect_pts[idx] = false;
        }
      });
    }
    //累加hessian & error
    double total_res = 0;
    int effective_num = 0;

    std::pair<Mat6d, Vec6d> H_and_error = std::accumulate(
        index.begin(), index.end(),
        std::pair<Mat6d, Vec6d>(Mat6d::Zero(), Vec6d::Zero()),
        [&jacobians, &errors, &effect_pts, &total_res,
         &effective_num](const std::pair<Mat6d, Vec6d> &pre,
                         int idx) -> std::pair<Mat6d, Vec6d> {
          if (!effect_pts[idx]) {
            return pre;
          } else {
            total_res += errors[idx].dot(errors[idx]);
            effective_num++;

            //用雅可比矩阵的乘积求和近似替代海森矩阵
            Mat6d H_sum =
                pre.first + jacobians[idx].transpose() * jacobians[idx];
            Vec6d error_sum =
                pre.second - jacobians[idx].transpose() * errors[idx];

            return std::pair<Mat6d, Vec6d>(H_sum, error_sum);
          }
        });

    if (effective_num < options_.min_effective_pts_) {
      LOG(WARNING) << "effective num too small: " << effective_num;
      return false;
    }

    Mat6d H = H_and_error.first;
    Vec6d b = H_and_error.second;

    Vec6d dx = H.inverse() * b;
    // update init pose 更新高斯牛顿法的步长
    pose.so3() = pose.so3() * SO3::exp(dx.head<3>()); // 相当于右乘一个旋转扰动
    pose.translation() += dx.tail<3>();

    // 更新
    LOG(INFO) << "iter " << iter << " total res: " << total_res
              << ", eff: " << effective_num
              << ", mean res: " << total_res / effective_num
              << ", dxn: " << dx.norm();

    if (gt_set_) {
      double pose_error = (gt_pose_.inverse() * pose).log().norm();
      LOG(INFO) << "iter " << iter << " pose error: " << pose_error;
    }

    if (dx.norm() < options_.eps_) {
      LOG(INFO) << "converged, dx = " << dx.transpose();
      break;
    }
  }

  init_pose = pose;
  return true;
}

bool ICP_3D::AlignP2Line(SE3 &init_pose) {
  LOG(INFO) << "aligning with point to line";
  assert(target_ != nullptr && source_ != nullptr);

  SE3 pose = init_pose;
  if (!options_.use_initial_translation_) {
    pose.translation() = target_center_ - source_center_; // 设置平移初始值
  }
  LOG(INFO) << "init guess pose: " << pose;

  // 对点的索引，预先生成
  std::vector<int> index(source_->points.size());
  for (int i = 0; i < index.size(); ++i) {
    index[i] = i;
  }

  std::vector<bool> effect_pts(index.size(), false);
  std::vector<Eigen::Matrix<double, 3, 6>> jacobians(index.size());
  std::vector<Vec3d> errors(index.size());

  for (int iter = 0; iter < options_.max_iteration_; ++iter) {
// gauss-newton 迭代
#pragma omp parallel
    {
      std::for_each(index.begin(), index.end(), [&](int idx) {
        auto q = ToVec3d(source_->points[idx]);
        Vec3d qs = pose * q; // 转换之后的q
        std::vector<int> nn;
        kd_tree_->GetClosestPoint(ToPointType(qs), nn, 5); // 这里取5个最近邻
        if (nn.size() == 5) {
          // convert to eigen
          std::vector<Vec3d> nn_eigen;
          for (int i = 0; i < 5; ++i) {
            nn_eigen.emplace_back(ToVec3d(target_->points[nn[i]]));
          }

          Vec3d d, p0; // p0 数据中心, d空间直线方向向量
          if (!math::FitLine(nn_eigen, p0, d, options_.max_line_distance_)) {
            // 失败的不要
            effect_pts[idx] = false;
            return;
          }

          // 点到直线距离 A x B 等价与 (A^) * B
          Vec3d err = SO3::hat(d) * (qs - p0);
          // LOG(INFO) << "err: " << err;

          // 叉乘计算点到直线距离
          // Vec3d test_p = qs - p0;
          // Vec3d test_d_norm = d.normalized();
          // Vec3d test_err = test_d_norm.cross(test_p);
          // LOG(INFO) << "test_err: " << test_err;

          if (err.norm() > options_.max_line_distance_) {
            // 点离的太远了不要
            effect_pts[idx] = false;
            return;
          }

          effect_pts[idx] = true;

          // build residual
          Eigen::Matrix<double, 3, 6> J;
          // 这里仍使用对右乘扰动求导，与点到点一样，只是左边多了SO3::hat(d)这项
          J.block<3, 3>(0, 0) =
              -SO3::hat(d) * pose.so3().matrix() * SO3::hat(q);
          J.block<3, 3>(0, 3) = SO3::hat(d);

          jacobians[idx] = J;
          errors[idx] = err;
        } else {
          effect_pts[idx] = false;
        }
      });
    }

    // 累加Hessian和error,计算dx
    // 原则上可以用reduce并发，写起来比较麻烦，这里写成accumulate
    double total_res = 0;
    int effective_num = 0;
    auto H_and_err = std::accumulate(
        index.begin(), index.end(),
        std::pair<Mat6d, Vec6d>(Mat6d::Zero(), Vec6d::Zero()),
        [&jacobians, &errors, &effect_pts, &total_res,
         &effective_num](const std::pair<Mat6d, Vec6d> &pre,
                         int idx) -> std::pair<Mat6d, Vec6d> {
          if (!effect_pts[idx]) {
            return pre;
          } else {
            total_res += errors[idx].dot(errors[idx]);
            effective_num++;
            return std::pair<Mat6d, Vec6d>(
                pre.first + jacobians[idx].transpose() * jacobians[idx],
                pre.second - jacobians[idx].transpose() * errors[idx]);
          }
        });

    if (effective_num < options_.min_effective_pts_) {
      LOG(WARNING) << "effective num too small: " << effective_num;
      return false;
    }

    Mat6d H = H_and_err.first;
    Vec6d err = H_and_err.second;

    Vec6d dx = H.inverse() * err;
    pose.so3() = pose.so3() * SO3::exp(dx.head<3>());
    pose.translation() += dx.tail<3>();

    if (gt_set_) {
      double pose_error = (gt_pose_.inverse() * pose).log().norm();
      LOG(INFO) << "iter " << iter << " pose error: " << pose_error;
    }

    // 更新
    LOG(INFO) << "iter " << iter << " total res: " << total_res
              << ", eff: " << effective_num
              << ", mean res: " << total_res / effective_num
              << ", dxn: " << dx.norm();

    if (dx.norm() < options_.eps_) {
      LOG(INFO) << "converged, dx = " << dx.transpose();
      break;
    }
  }

  init_pose = pose;
  return true;
}

bool ICP_3D::AlignP2Plane(SE3 &init_pose) {
  LOG(INFO) << "aligning with point to plane";
  assert(target_ != nullptr && source_ != nullptr);
  // 整体流程与p2p一致，读者请关注变化部分

  SE3 pose = init_pose;
  if (!options_.use_initial_translation_) {
    pose.translation() = target_center_ - source_center_; // 设置平移初始值
  }

  std::vector<int> index(source_->points.size());
  for (int i = 0; i < index.size(); ++i) {
    index[i] = i;
  }

  std::vector<bool> effect_pts(index.size(), false);
  std::vector<Eigen::Matrix<double, 1, 6>> jacobians(index.size());
  std::vector<double> errors(index.size());

  for (int iter = 0; iter < options_.max_iteration_; ++iter) {
// gauss-newton 迭代
#pragma omp parallel
    {
      std::for_each(index.begin(), index.end(), [&](int idx) {
        auto q = ToVec3d(source_->points[idx]);
        Vec3d qs = pose * q; // 转换之后的q
        std::vector<int> nn;
        kd_tree_->GetClosestPoint(ToPointType(qs), nn, 5); // 这里取5个最近邻
        if (nn.size() > 3) {
          // convert to eigen
          std::vector<Vec3d> nn_eigen;
          for (int i = 0; i < nn.size(); ++i) {
            nn_eigen.emplace_back(ToVec3d(target_->points[nn[i]]));
          }

          Vec4d n; // 平面法向量
          if (!math::FitPlane(nn_eigen, n)) {
            // 失败的不要
            effect_pts[idx] = false;
            return;
          }
          // 点到平面距离，把点带入平面方程
          double dis = n.head<3>().dot(qs) + n[3];
          if (fabs(dis) > options_.max_plane_distance_) {
            // 点离的太远了不要
            effect_pts[idx] = false;
            return;
          }

          effect_pts[idx] = true;

          // build residual
          // 这里与点到线的计算jacobian一样
          Eigen::Matrix<double, 1, 6> J;
          J.block<1, 3>(0, 0) =
              -n.head<3>().transpose() * pose.so3().matrix() * SO3::hat(q);
          J.block<1, 3>(0, 3) = n.head<3>().transpose();

          jacobians[idx] = J;
          errors[idx] = dis;
        } else {
          effect_pts[idx] = false;
        }
      });
    }

    // 累加Hessian和error,计算dx
    // 原则上可以用reduce并发，写起来比较麻烦，这里写成accumulate
    double total_res = 0;
    int effective_num = 0;
    auto H_and_err = std::accumulate(
        index.begin(), index.end(),
        std::pair<Mat6d, Vec6d>(Mat6d::Zero(), Vec6d::Zero()),
        [&jacobians, &errors, &effect_pts, &total_res,
         &effective_num](const std::pair<Mat6d, Vec6d> &pre,
                         int idx) -> std::pair<Mat6d, Vec6d> {
          if (!effect_pts[idx]) {
            return pre;
          } else {
            total_res += errors[idx] * errors[idx];
            effective_num++;
            return std::pair<Mat6d, Vec6d>(
                pre.first + jacobians[idx].transpose() * jacobians[idx],
                pre.second - jacobians[idx].transpose() * errors[idx]);
          }
        });

    if (effective_num < options_.min_effective_pts_) {
      LOG(WARNING) << "effective num too small: " << effective_num;
      return false;
    }

    Mat6d H = H_and_err.first;
    Vec6d err = H_and_err.second;

    Vec6d dx = H.inverse() * err;
    pose.so3() = pose.so3() * SO3::exp(dx.head<3>());
    pose.translation() += dx.tail<3>();

    // 更新
    LOG(INFO) << "iter " << iter << " total res: " << total_res
              << ", eff: " << effective_num
              << ", mean res: " << total_res / effective_num
              << ", dxn: " << dx.norm();

    if (gt_set_) {
      double pose_error = (gt_pose_.inverse() * pose).log().norm();
      LOG(INFO) << "iter " << iter << " pose error: " << pose_error;
    }

    if (dx.norm() < options_.eps_) {
      LOG(INFO) << "converged, dx = " << dx.transpose();
      break;
    }
  }

  init_pose = pose;
  return true;
}

} // namespace sad

#endif
