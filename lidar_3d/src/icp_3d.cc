#ifdef ROS_CATKIN

#include "icp_3d.h"
#include "math_utils.h"

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

} // namespace sad

#endif
