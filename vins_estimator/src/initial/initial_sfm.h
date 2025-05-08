#ifndef VINS_ESTIMATOR_INITIAL_SFM
#define VINS_ESTIMATOR_INITIAL_SFM

#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include <cstdlib>
#include <deque>
#include <eigen3/Eigen/Dense>
#include <iostream>
#include <map>
#include <opencv2/core/eigen.hpp>
#include <opencv2/opencv.hpp>

// SfM, Structure from Motion
// 使用了 Ceres Solver
// 进行非线性优化，用于在初始化阶段估计相机位姿和稀疏三维地图点

namespace sensor_lab {

// 描述一个特征点在整个 SfM 中的信息
// 包含其在不同帧中的二维观测以及估算出的世界坐标系下的三维位置
struct SFMFeature {
  bool state; // 该点是否已完成三角化
  int id;     // 特征点ID
  std::vector<std::pair<int, Eigen::Vector2d>>
      observation;    // 每一帧中该特征的观测（帧号+二维坐标）
  double position[3]; // 三维位置
  double depth;       // 深度值
};

// 重投影误差残差块的优化
struct ReprojectionError3D {
  // constructor 接受观测到的像素坐标
  ReprojectionError3D(double observed_u, double observed_v)
      : observed_u(observed_u), observed_v(observed_v) {}

  // 将世界坐标系下点投影到相机坐标系下并进行归一化投影
  template <typename T>
  bool operator()(const T *const camera_R, const T *const camera_T,
                  const T *point, T *residuals) const {
    T p[3];
    // 将世界坐标系下的点 point 转换到相机坐标系下 p
    ceres::QuaternionRotatePoint(camera_R, point, p);
    p[0] += camera_T[0];
    p[1] += camera_T[1];
    p[2] += camera_T[2];
    // 归一化投影
    T xp = p[0] / p[2];
    T yp = p[1] / p[2];

    // 计算重投影误差 预测-观测
    residuals[0] = xp - T(observed_u);
    residuals[1] = yp - T(observed_v);
    return true;
  }

  // 创建一个 Ceres 的自动微分残差带价函数
  static ceres::CostFunction *Create(const double observed_x,
                                     const double observed_y) {
    // ReprojectionError3D: 代价函数的实现结构体
    // <误差模型, 残差维度, 参数1维度, 参数2维度, 参数3维度> 参考 operator()
    // 函数项 2: 残差维度，表示每个误差项有两个输出（图像坐标u和v的误差） 4:
    // 第一个可优化参数块的维度，这里是相机旋转（四元数表示，4维） 3:
    // 第二个可优化参数块的维度，这里是相机平移（3维向量） 3:
    // 第三个可优化参数块的维度，这里是三维空间点（x, y, z）
    return (new ceres::AutoDiffCostFunction<ReprojectionError3D, 2, 4, 3, 3>(
        new ReprojectionError3D(observed_x, observed_y)));
  }

  double observed_u;
  double observed_v;
};

class GlobalSFM {
public:
  GlobalSFM();
  ~GlobalSFM();

  /// @brief 在初始化阶段完成 运动恢复结构 SFM 任务
  /// @note
  /// 从滑动窗口中的多个图像帧及其特征点观测中，联合估计相机位姿和稀疏三维地图点
  /// @param frame_num 当前滑动窗口中的帧数
  /// @param q 输出的相机旋转和平移数组（每个帧一个）
  /// @param T
  /// @param l 参考帧索引
  /// @param relative_R 参考帧到最后一帧的相对旋转和平移（来自预积分或IMU）
  /// @param relative_T
  /// @param sfm_f 所有特征点的信息（包含二维观测）
  /// @param sfm_tracked_points 输出的重建后的三维地图点集合
  /// @return
  bool construct(int frame_num, Eigen::Quaterniond *q, Eigen::Vector3d *T,
                 int l, const Eigen::Matrix3d relative_R,
                 const Eigen::Vector3d relative_T,
                 Eigen::vector<SFMFeature> &sfm_f,
                 std::map<int, Eigen::Vector3d> &sfm_tracked_points);

private:
  /// @brief 基于三角化点求解相机位姿
  /// @param R_initial
  /// 输入的初始旋转矩阵（世界坐标系到相机坐标系），输出优化后的结果
  /// @param P_initial 输入的初始平移向量，输出优化后的结果
  /// @param i 当前需要求解位姿的帧索引
  /// @param sfm_f 所有特征点的集合，包含它们在各帧中的观测坐标和三维位置
  /// @return 更新输入的旋转和平移参数并返回成功与否
  bool solveFrameByPnP(Eigen::Matrix3d &R_initial, Eigen::Vector3d &P_initial,
                       int i, std::vector<SFMFeature> &sfm_f);

  // 给定两个相机位姿和对应的二维点，三角化得到三维点, 返回归一化后的三维坐标
  // SVD分解求三角化
  // https://blog.csdn.net/Walking_roll/article/details/119984469
  void triangulatePoint(Eigen::Matrix<double, 3, 4> &Pose0,
                        Eigen::Matrix<double, 3, 4> &Pose1,
                        Eigen::Vector2d &point0, Eigen::Vector2d &point1,
                        Eigen::Vector3d &point_3d);

  /// @brief 对两帧之间的所有共视点进行三角化
  /// @note 1. 初始化时从滑动窗口中选择两帧进行初始地图点重建; 2. 先用 PnP
  /// 解出新帧的位姿，再与参考帧一起三角化新点; 3.
  /// 提供初始地图点，供后续非线性优化使用
  /// @param frame0 需要进行三角化的两个图像帧索引
  /// @param Pose0 对应帧的相机投影矩阵（3x4），形式为 [R|t]
  /// @param frame1 需要进行三角化的两个图像帧索引
  /// @param Pose1 对应帧的相机投影矩阵（3x4），形式为 [R|t]
  /// @param sfm_f 所有特征点的集合
  void triangulateTwoFrames(int frame0, Eigen::Matrix<double, 3, 4> &Pose0,
                            int frame1, Eigen::Matrix<double, 3, 4> &Pose1,
                            std::vector<SFMFeature> &sfm_f);

  int feature_num; // 特征点数量
};

} // namespace sensor_lab

#endif
