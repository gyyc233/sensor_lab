#ifndef LIDAR_3D_INCREMENTAL_NDT_H
#define LIDAR_3D_INCREMENTAL_NDT_H
#ifdef ROS_CATKIN

#include "eigen_type/eigen_types.h"
#include "g2o_types/g2o_types.h"
#include "point_cloud/point_types.h"

#include <list>

namespace sad {

/**
 * 增量版本的NDT
 * 内部会维护增量式的voxels，自动删除较旧的voxel，往voxel里添加点云时，更新其均值和协方差估计
 */
class IncrementalNdt3D {
public:
  enum class NearbyType {
    CENTER,  // 只考虑中心
    NEARBY6, // 上下左右前后
  };

  struct Options {
    int max_iteration_ = 4;       // 最大迭代次数
    double voxel_size_ = 1.0;     // 体素大小
    double inv_voxel_size_ = 1.0; // 体素大小之逆
    int min_effective_pts_ = 10;  // 最近邻点数阈值
    int min_pts_in_voxel_ = 5;    // 每个栅格中最小点数
    int max_pts_in_voxel_ = 50;   // 每个栅格中最大点数
    double eps_ = 1e-3;           // 收敛判定条件
    double res_outlier_th_ = 5.0; // 异常值拒绝阈值
    size_t capacity_ = 50000;     // 缓存的体素数量

    NearbyType nearby_type_ = NearbyType::NEARBY6;
  };

  // KeyType 可以看做voxel id
  using KeyType = Eigen::Matrix<int, 3, 1>; // 体素的索引

  /// 体素内置结构
  struct VoxelData {
    VoxelData() {}
    VoxelData(const Vec3d &pt) {
      pts_.emplace_back(pt);
      num_pts_ = 1;
    }

    void addPoint(const Vec3d &pt) {
      pts_.emplace_back(pt);
      if (!ndt_estimated_) {
        num_pts_++;
      }
    }

    std::vector<Vec3d> pts_; // 内部点，多于一定数量之后再估计均值和协方差
    Vec3d mu_ = Vec3d::Zero();    // 均值
    Mat3d sigma_ = Mat3d::Zero(); // 协方差
    Mat3d info_ = Mat3d::Zero();  // 协方差之逆

    bool ndt_estimated_ = false; // NDT是否已经估计
    int num_pts_ = 0;            // 总共的点数，用于更新估计
  };

  IncrementalNdt3D();

  IncrementalNdt3D(Options options);

  /// 获取一些统计信息
  int numGrids() const;

  /// 在voxel里添加点云，
  void addCloud(CloudPtr cloud_world);

  /// 设置被配准的Scan
  void setSource(CloudPtr source);

  /// 使用gauss-newton方法进行ndt配准
  bool alignNdt(SE3 &init_pose);

  /**
   * 计算给定Pose下的雅可比和残差矩阵，符合IEKF中符号（8.17, 8.19）
   * @param pose
   * @param HTVH
   * @param HTVr
   */
  void computeResidualAndJacobians(const SE3 &pose, Mat18d &HTVH, Vec18d &HTVr);

  /**
   * 根据估计的NDT建立edges
   * @param v 输入参数，位姿顶点
   * @param edges 输出参数，全部的有效的NDT残差边
   */
  void buildNDTEdges(VertexPose *v, std::vector<EdgeNDT *> &edges);

private:
  /// 根据最近邻的类型，生成附近网格
  void generateNearbyGrids();

  /// 更新体素内部数据, 根据新加入的pts和历史的估计情况来确定自己的估计
  void updateVoxel(VoxelData &v);

  CloudPtr source_ = nullptr;
  Options options_;

  using KeyAndData = std::pair<KeyType, VoxelData>; // 预定义

  // LRU(least recently used 最少最近使用策略) 需要一个 hash map
  // 与一个双向链表进行维护
  std::list<KeyAndData> data_; // 真实数据，会缓存，也会清理
  // std::list 是双向链表容器
  std::unordered_map<KeyType, std::list<KeyAndData>::iterator, hash_vec<3>>
      grids_; // 栅格数据，存储真实数据的迭代器
  std::vector<KeyType> nearby_grids_; // 附近的栅格

  bool flag_first_scan_ = true; // 首帧点云特殊处理
};
} // namespace sad

#endif
#endif
