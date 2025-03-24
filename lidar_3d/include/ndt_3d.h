#ifndef LIDAR_3D_NDT_3D_H
#define LIDAR_3D_NDT_3D_H
#ifdef ROS_CATKIN

#include "eigen_type/eigen_types.h"
#include "point_cloud/point_types.h"
#include "unordered_map"

namespace sad {

/**
 * 3D 形式的NDT
 */
class NDT_3D {
public:
  enum class NearbyTYpe {
    CENTER,  // 只考虑中心
    NEARBY6, // 上下左右前后
  };

  struct Options {
    int max_iteration_ = 20;       // 最大迭代次数
    double voxel_size_ = 1.0;      // 体素大小
    double inv_voxel_size_ = 1.0;  //
    int min_effective_pts_ = 10;   // 最近邻点数阈值
    int min_pts_in_voxel_ = 3;     // 每个栅格中最小点数
    double eps_ = 1e-2;            // 收敛判定条件
    double res_outlier_th_ = 20.0; // 异常值拒绝阈值
    bool remove_centroid_ = false; // 是否计算两个点云中心并移除中心？

    NearbyType nearby_type_ = NearbyType::NEARBY6;
  };

  using KeyType = Eigen::Matrix<int, 3, 1>; // 体素的索引

  struct VoxelData {
    VoxelData() {}
    VoxelData(size_t id) { idx_.emplace_back(id); }

    std::vector<size_t> idx_;     // 点云中点的索引
    Vec3d mu_ = Vec3d::Zero();    // 均值
    Mat3d sigma_ = Mat3d::Zero(); // 协方差
    Mat3d info_ = Mat3d::Zero();  // 协方差的逆，信息矩阵
  };

  NDT_3D();

  ~NDT_3D();

  NDT_3D(Options options);

  void setTarget(CloudPtr target);

  void setSource(CloudPtr source);

  void setGtPose(const SE3 &gt_pose);

  /// @brief 将source配准到target
  /// @param init_pose
  /// @return
  bool alignNewtonGaussianNdt(SE3 &init_pose);

private:
  void buildVoxels();

  void generateNearbyGrids();

  CloudPtr target_ = nullptr;
  CloudPtr source_ = nullptr;

  Vec3d target_center_ = Vec3d::Zero();
  Vec3d source_center_ = Vec3d::Zero();

  SE3 gt_pose_;
  bool gt_set_ = false;

  Options options_;

  // key: KeyType, value: VoxelData
  std::unordered_map<KeyType, VoxelData, hash_vec<3>> grids_; // 栅格数据
  std::vector<KeyType> nearby_grids_;                         // 附近的栅格
};
} // namespace sad

#endif
#endif
