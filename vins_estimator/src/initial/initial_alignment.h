#ifndef VINS_ESTIMATOR_INITIAL_ALIGNMENT
#define VINS_ESTIMATOR_INITIAL_ALIGNMENT

#include "../factor/imu_factor.h"
#include "../feature_manager.h"
#include "../utility/utility.h"
#include <eigen3/Eigen/Dense>
#include <iostream>
#include <map>

namespace sensor_lab {
class ImageFrame {
public:
  ImageFrame(){};
  ImageFrame(const std::map<
                 int, std::vector<std::pair<int, Eigen::Matrix<double, 7, 1>>>>
                 &_points,
             double _t)
      : t{_t}, is_key_frame{false} {
    points = _points;
  };

  // {{特征点id,[<相机索引,特征点信息(x,y,z,u,v,velocity_x,
  // velocity_y)>,<>,<>]},{},{}}
  // 该结构便于在多相机、多帧之间进行特征匹配与三角化
  std::map<int, std::vector<std::pair<int, Eigen::Matrix<double, 7, 1>>>>
      points; // 图像特征点数据。每个 feature id 对应多个观测
  double t;   // 当前帧时间戳
  Eigen::Matrix3d R; // 当前帧相对于世界坐标系的旋转矩阵
  Eigen::Vector3d T; // 当前帧相对于世界坐标系的平移向量
  IntegrationBase *pre_integration; // IMU 预积分数据指针
  bool is_key_frame;                // 是否为关键帧
};

/// @brief 视觉与 IMU 的联合标定与初始化
/// @note 包含以下任务: 1. 陀螺仪偏置校正（Gyro Bias Correction）2.
/// 重力向量优化（Gravity Optimization）3. 尺度恢复（Scale Recovery）4.
/// 绝对尺度对齐（Aligning to World Frame）
/// @param all_image_frame 所有图像帧的集合，按时间排序
/// @param Bgs output 陀螺仪偏置
/// @param g  output 重力向量
/// @param x output 优化后的状态向量（包含位姿、速度、偏差等）
/// @return
bool VisualIMUAlignment(std::map<double, ImageFrame> &all_image_frame,
                        Eigen::Vector3d *Bgs, Eigen::Vector3d &g,
                        Eigen::VectorXd &x);
} // namespace sensor_lab

#endif
