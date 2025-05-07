#include "utility.h"

namespace sensor_lab {
Eigen::Matrix3d Utility::g2R(const Eigen::Vector3d &g) {
  Eigen::Matrix3d R0;
  Eigen::Vector3d ng1 = g.normalized(); // 归一化为单位向量,只保留方向信息
  Eigen::Vector3d ng2(0.0, 0.0, 1.0); // 表示目标坐标系的上方向(z轴）

  // 构建旋转矩阵，从ng1到ng2, 两个向量都穿过原点
  R0 = Eigen::Quaterniond::FromTwoVectors(ng1, ng2).toRotationMatrix();

  // 旋转矩阵转换为欧拉角（yaw, pitch, roll），并取第一个值（即
  // yaw）作为当前偏航角
  double yaw = Utility::R2ypr(R0).x();

  // 绕 yaw 轴反向旋转,消除当前旋转矩阵中的偏航角影响，使最终方向对齐世界坐标系
  // 左乘，绕坐标系z轴旋转－yaw, 使得x y轴与坐标系x y轴投影对齐
  R0 = Utility::ypr2R(Eigen::Vector3d{-yaw, 0, 0}) * R0;
  return R0;
}
} // namespace sensor_lab
