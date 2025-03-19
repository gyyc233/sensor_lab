#ifndef LIDAR_3D_ICP_3D_H
#define LIDAR_3D_ICP_3D_H
#ifdef ROS_CATKIN

#include "eigen_type/eigen_types.h"
#include "pcl_test/include/kd_tree.h"
#include "point_cloud/point_types.h"
#include <memory>

namespace sad {
/**
 * 3D 形式的ICP
 * 先SetTarget, 再SetSource, 然后调用Align方法获取位姿
 *
 * ICP 求解R,t 将source点云配准到target点云上
 * 如果 p 是source点云中的点，那么R*p+t就得到target中的配对点
 *
 * 使用第pcl_test的Kd树来求取最近邻
 */
class ICP_3D {
public:
  struct Options {
    int max_iteration_ = 20;           // 最大迭代次数
    double max_nn_distance_ = 1.0;     // 点到点最近邻查找阈值
    double max_plane_distance_ = 0.05; // 平面最近邻查找时阈值
    double max_line_distance_ = 0.5;   // 点线最近邻查找时阈值
    int min_effective_pts_ = 10;       // 最近邻点数阈值
    double eps_ = 1e-2;                // 收敛判定条件
    bool use_initial_translation_ = false; // 是否使用初始位姿中的平移估计
  };

  ICP_3D() = delate;

  ICP_3D(Options optional);

  ~ICP_3D();

  void setTarget(pcl::PointCloud<pcl::PointXYZI>::Ptr target);

  void setSource(pcl::PointCloud<pcl::PointXYZI>::Ptr source);

  /// @brief
  /// 允许用户为配准算法提供一个基准位姿，以便在配准完成后，可以通过比较估计位姿和地面真实位姿之间的差异来评估算法的性能。
  /// @param gt_pose
  void setGroundTruth(const SE3 &gt_pose);

  /// @brief 使用gauss-newton方法进行配准, 点到点
  /// @param init_pose 为ICP 提供初值
  /// @return
  bool AlignP2P(SE3 &init_pose);

  /// @brief 使用gauss-newton方法进行配准, 点到线
  /// @param init_pose 为ICP 提供初值
  /// @return
  bool AlignP2Line(SE3 &init_pose);

  /// @brief 使用gauss-newton方法进行配准, 点到面
  /// @param init_pose 为ICP 提供初值
  /// @return
  bool AlignP2Plane(SE3 &init_pose);

private:
  void buildTargetKdTree();
  std::shared_ptr<KdTree> kd_tree_;
  pcl::PointCloud<pcl::PointXYZI>::Ptr target_;
  pcl::PointCloud<pcl::PointXYZI>::Ptr source_;

  Eigen::Vector3d target_center_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d source_center_ = Eigen::Vector3d::Zero();
  bool gt_set_ = false; // 真值是否设置
  SE3 gt_pose_;

  Options options_;
};
} // namespace sad

#endif
#endif
