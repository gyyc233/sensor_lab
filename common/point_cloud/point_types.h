#ifndef COMMON_POINT_CLOUD_POINT_TYPES_H
#define COMMON_POINT_CLOUD_POINT_TYPES_H

#define PCL_NO_PRECOMPILE
#include <pcl/impl/pcl_base.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "eigen_type/eigen_types.h"

namespace sad {

// 定义系统中用到的点和点云类型
using PointType = pcl::PointXYZI;
using PointCloudType = pcl::PointCloud<PointType>;
using CloudPtr = PointCloudType::Ptr;
using PointVec = std::vector<PointType, Eigen::aligned_allocator<PointType>>;
using IndexVec = std::vector<int>;

// 点云到Eigen的常用的转换函数
inline Vec3f ToVec3f(const PointType &pt) { return pt.getVector3fMap(); }
inline Vec3d ToVec3d(const PointType &pt) {
  return pt.getVector3fMap().cast<double>();
}

// 模板类型转换函数
template <typename T, int dim>
inline Eigen::Matrix<T, dim, 1> ToEigen(const PointType &pt);

template <>
inline Eigen::Matrix<float, 2, 1> ToEigen<float, 2>(const PointType &pt) {
  return Vec2f(pt.x, pt.y);
}

template <>
inline Eigen::Matrix<float, 3, 1> ToEigen<float, 3>(const PointType &pt) {
  return Vec3f(pt.x, pt.y, pt.z);
}

template <typename S>
inline PointType ToPointType(const Eigen::Matrix<S, 3, 1> &pt) {
  PointType p;
  p.x = pt.x();
  p.y = pt.y();
  p.z = pt.z();
  return p;
}

/// 带ring, range等其他信息的全量信息点云
struct FullPointType {
  PCL_ADD_POINT4D;
  float range = 0;
  float radius = 0;
  uint8_t intensity = 0; // 反射强度
  uint8_t ring = 0;      // 探头数量
  uint8_t angle = 0;
  double time = 0;
  float height = 0;

  inline FullPointType() {}
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

/// 全量点云的定义
using FullPointCloudType = pcl::PointCloud<FullPointType>;
using FullCloudPtr = FullPointCloudType::Ptr;

inline Vec3f ToVec3f(const FullPointType &pt) { return pt.getVector3fMap(); }
inline Vec3d ToVec3d(const FullPointType &pt) {
  return pt.getVector3fMap().cast<double>();
}

/// ui中的点云颜色
using UiPointType = pcl::PointXYZRGBA;
using UiPointCloudType = pcl::PointCloud<UiPointType>;
using UiCloudPtr = UiPointCloudType::Ptr;

} // namespace sad

#endif
