#ifndef LIDAR_2D_ICP_2D_H
#define LIDAR_2D_ICP_2D_H

#ifdef ROS_CATKIN
#include "eigen_type/eigen_types.h"
#include "navigation_and_mapping/lidar_utils.h"

#include <pcl/search/kdtree.h>

namespace sad {

/**
 * ICP实现
 * 用法：先SetTarget, 此时构建target点云的KD树；再SetSource，然后调用Align*方法
 */
class Icp2d {
public:
  using Point2d = pcl::PointXY;
  using Cloud2d = pcl::PointCloud<Point2d>;
  Icp2d() {}
  ~Icp2d() {}

  /// @brief 设置目标的Scan
  /// @param target
  void setTarget(Scan2d::Ptr target) {
    target_scan_ = target;
    BuildTargetKdTree();
  }

  /// @brief 设置被配准的Scan
  /// @param source
  void setSource(Scan2d::Ptr source) { source_scan_ = source; }

  /// @brief 使用高斯牛顿法进行配准
  /// @param init_pose 大概准确的初始值
  /// @return
  bool alignGaussNewton(SE2 &init_pose);

  /// @brief 使用高斯牛顿法进行配准, Point-to-Plane
  /// @param init_pose 大概准确的初始值
  /// @return
  bool alignGaussNewtonPoint2Plane(SE2 &init_pose);

private:
  /// @brief 建立目标点云的Kdtree
  void buildTargetKdTree();

  pcl::search::KdTree<Point2d> kd_tree_;
  Cloud2d::Ptr target_cloud_; // PCL 形式的target cloud

  Scan2d::Ptr target_scan_ = nullptr;
  Scan2d::Ptr source_scan_ = nullptr;
};

} // namespace sad

#endif
#endif // LIDAR_2D_ICP_2D_H
