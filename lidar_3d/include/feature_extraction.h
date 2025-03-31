#ifndef LIDAR_3D_FEATURE_EXTRACTION_H
#define LIDAR_3D_FEATURE_EXTRACTION_H
#ifdef ROS_CATKIN

#include "point_cloud/point_types.h"

namespace sad {

/**
 * 根据线束信息来提取特征
 * 需要知道雷达的线数分布，目前只支持velodyne的雷达
 */
class FeatureExtraction {

  /// @brief 某个线束的id和曲率
  struct IdAndValue {
    IdAndValue() {}
    IdAndValue(int id, double value) : id_(id), value_(value) {}
    int id_ = 0;
    double value_ = 0; // 曲率
  };

public:
  FeatureExtraction();

  ~FeatureExtraction();

  /**
   * 提取角点和平面点
   * @param pc_in         输入点云（全量信息）
   * @param pc_out_edge   输出角点的点云
   * @param pc_out_surf   输出平面的点云
   */
  void extract(FullCloudPtr pc_in, CloudPtr &pc_out_edge,
               CloudPtr &pc_out_surf);

  /**
   * 对单独一段区域提取角点和面点
   * @param pc_in 线束中的某条线
   * @param cloud_curvature　pc_in　中某个连续区间的点特征
   * @param pc_out_edge 区间的角点的点云
   * @param pc_out_surf 区间的平面点云
   */
  void extractFromSector(const CloudPtr &pc_in,
                         std::vector<IdAndValue> &cloud_curvature,
                         CloudPtr &pc_out_edge, CloudPtr &pc_out_surf);
};
} // namespace sad

#endif
#endif
