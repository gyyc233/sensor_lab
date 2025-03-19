#ifdef ROS_CATKIN

#include "icp_3d.h"
#include "math_utils.h"

namespace sad {
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
                      Eigen::Vector3d::zero().eval(),
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

      if (!nn.empty) {
        // target中的p是 qs(source)的最邻近点
        Vec3d p = ToVec3d(target_->points[nn[0]]);
        double dis2 = (p - ps).squaredNorm();
        if (dis2 > options_.max_nn_distance_) {
          effect_pts[idx] = false;
          continue;
        }
        effect_pts[idx] = true;

        // build residual
        Vec3d e = p - qs;
        // 因为假设是source经过rt变换到target 问题描述为 e = |p_t - (R*p_s +
        // T)|取最小 e分别对位姿的旋转和平移部分求偏导 则jacobian为 3*6
        // 对旋转求导使用李代数se(3)的左乘扰动模型
        Eigen::Matrix<double, 3, 6> J;
        J.block<3, 3>(0, 0) = -pose.so3().matrix() * SO3::hat(q);
        J.block<3, 3>(0, 3) = Mat3d::Identity();
        J = -1 * J;

        jacobians[idx] = J;
        errors[idx] = e;
      } else {
        effect_pts[idx] = false;
      }
    });
  }
}

} // namespace sad

#endif