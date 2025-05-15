#ifndef VINS_ESTIMATOR_FEATURE_MANAGER
#define VINS_ESTIMATOR_FEATURE_MANAGER
#ifndef ROS_CATKIN
#define ROS_CATKIN

#include <algorithm>
#include <list>
#include <numeric>
#include <vector>

#include <eigen3/Eigen/Dense>

#include "parameters.h"

// 定义了特征点管理模块的核心类和数据结构
// 负责在视觉惯性里程计（VIO）系统中对特征点进行生命周期管理、深度估计、滑动窗口更新等操作

namespace sensor_lab {
/// @brief 表示一个特征点在某一帧中的观测信息
class FeaturePerFrame {
public:
  FeaturePerFrame(const Eigen::Matrix<double, 7, 1> &_point, double td) {
    point.x() = _point(0);
    point.y() = _point(1);
    point.z() = _point(2);
    uv.x() = _point(3);
    uv.y() = _point(4);
    velocity.x() = _point(5);
    velocity.y() = _point(6);
    cur_td = td;
  }
  double cur_td;         // 时间偏移校正参数（时间戳差值）
  Eigen::Vector3d point; // 特征点在相机坐标系下的空间坐标 (x, y, z)
  Eigen::Vector2d uv;    // 归一化图像坐标或像素坐标
  Eigen::Vector2d velocity; // 像素速度（用于运动补偿）
  double z;                 // 深度值（初始化后使用）
  bool is_used;             // 是否被三角化使用过
  double parallax;          // 视差，用于判断是否可用于初始化
  Eigen::MatrixXd A;   // 构造 Ax = b 的 A 部分（用于求解深度）
  Eigen::VectorXd b;   // 构造 Ax = b 的 b 部分
  double dep_gradient; // 深度梯度
};

// 表示一个唯一 ID 的特征点在整个生命周期中的信息
class FeaturePerId {
public:
  const int feature_id; // 特征点的唯一ID（由 FeatureTracker 分配）
  int start_frame;      // 首次观测到该特征点的帧号
  std::vector<FeaturePerFrame> feature_per_frame; // 特征点在每帧中的观测信息

  int used_num; // 被使用的次数（用于判断是否满足三角化条件）
  bool is_outlier;        // 是否为异常点
  bool is_margin;         // 是否被边缘化
  double estimated_depth; // 估计的深度值
  int solve_flag; // 深度求解状态（0:未解，1:成功，2:失败）

  Eigen::Vector3d gt_p; // 真实位置

  FeaturePerId(int _feature_id, int _start_frame)
      : feature_id(_feature_id), start_frame(_start_frame), used_num(0),
        estimated_depth(-1.0), solve_flag(0) {}

  // 返回最后一次观测该特征点的帧号
  int endFrame();
};

// 将不同帧中属于同一特征点的观测组织在一起，用于三角化、优化、滑动窗口更新等
class FeatureManager {
public:
  FeatureManager(Eigen::Matrix3d _Rs[]);

  /// @brief 设置多个相机到IMU的外参旋转矩阵
  /// @param _ric
  void setRic(Eigen::Matrix3d _ric[]);

  /// @brief 清除所有特征点
  void clearState();

  /// @brief 获取当前有效特征点数量
  /// @return
  int getFeatureCount();

  /// @brief 添加新特征点并检查视差是否足够用于初始化
  /// @note 1. 添加新的特征点到特征管理器; 2.
  /// 维护每个特征点在多帧中的观测信息; 3.
  /// 计算并判断视差是否足够大，以决定当前帧是否适合进行初始化（如三角化、位姿估计等）
  /// @param frame_count 当前帧编号（用于时间戳对齐）
  /// @param image 输入的特征点数据：key 是 feature_id，value 是每帧的观测信息
  /// @param td 时间偏移校正参数（用于去畸变或时间同步）
  /// @return
  bool addFeatureCheckParallax(
      int frame_count,
      const std::map<
          int, std::vector<std::pair<int, Eigen::Matrix<double, 7, 1>>>> &image,
      double td);

  void debugShow();

  /// @brief 获取两帧之间的对应点对
  /// @param frame_count_l
  /// @param frame_count_r
  /// @return
  std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>>
  getCorresponding(int frame_count_l, int frame_count_r);

  ///@param x 根据输入的向量 x 更新特征点的深度值。这个输入向量 x
  ///通常来自于后端优化器的结果，其中包含了每个有效特征点的逆深度值（inverse
  /// depth）
  void setDepth(const Eigen::VectorXd &x);

  // 移除深度求解失败的点
  void removeFailures();

  // 清除某些深度值
  void clearDepth(const Eigen::VectorXd &x);

  // 收集所有有效特征点的深度信息，
  // 构建一个深度向量，供后端优化器使用
  Eigen::VectorXd getDepthVector();

  // 三角化计算深度
  void triangulate(Eigen::Vector3d Ps[], Eigen::Vector3d tic[],
                   Eigen::Matrix3d ric[]);

  /// @brief 滑动窗口：移除旧帧，将3D点转到新参考帧下
  /// @note 滑动窗口机制限制了历史帧数量,
  /// 边缘化会把最老帧的状态从优化问题中移除,
  /// 但某些特征点可能还存在于后续帧中，不能直接丢弃,
  /// 将这些点更新到新参考帧下，避免重复三角化
  /// @param marg_R 被边缘化的旧参考帧在world下的的旋转矩阵
  /// @param marg_P 被边缘化的旧参考帧在world下的的平移向量
  /// @param new_R 新参考帧的旋转矩阵
  /// @param new_P 新参考帧的平移向量
  void removeBackShiftDepth(Eigen::Matrix3d marg_R, Eigen::Vector3d marg_P,
                            Eigen::Matrix3d new_R, Eigen::Vector3d new_P);

  /// @brief 从滑动窗口中移除最老一帧对特征点的所有观测信息
  void removeBack();

  // 移除指定帧之前的观测
  void removeFront(int frame_count);

  // 移除异常点
  void removeOutlier();

  std::list<FeaturePerId> feature; // 所有特征点
  int last_track_num;              // 上一帧追踪到的特征点数量

private:
  /// @brief 计算特征点在倒数第二帧和倒数第三帧之间的视差
  /// @param it_per_id 某个特征点在整个滑动窗口中的观测历史
  /// @param frame_count 当前处理的是第几帧（即当前窗口中最新的帧号）
  /// @return
  double compensatedParallax2(const FeaturePerId &it_per_id, int frame_count);

  const Eigen::Matrix3d *Rs;       // 每一帧中 IMU 的旋转矩阵
  Eigen::Matrix3d ric[NUM_OF_CAM]; // 第i个相机到IMU的外参旋转矩阵
};
} // namespace sensor_lab

#endif
#endif
