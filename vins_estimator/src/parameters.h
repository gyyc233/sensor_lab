#ifndef VINS_ESTIMATOR_PARAMETERS
#define VINS_ESTIMATOR_PARAMETERS

#include "utility/utility.h"
#include <eigen3/Eigen/Dense>
#include <fstream>
#include <opencv2/core/eigen.hpp>
#include <opencv2/opencv.hpp>
#include <vector>

namespace sensor_lab {
const double FOCAL_LENGTH = 460.0;
const int WINDOW_SIZE = 10; // 滑动窗口优化中保留的帧数
const int NUM_OF_CAM = 1;
const int NUM_OF_F = 1000; // 最多跟踪或存储的特征点数量
//#define UNIT_SPHERE_ERROR

extern double INIT_DEPTH;
extern double MIN_PARALLAX;
extern int ESTIMATE_EXTRINSIC;

extern double ACC_N, ACC_W;
extern double GYR_N, GYR_W;

// transform of imu to camera
extern std::vector<Eigen::Matrix3d> RIC;
extern std::vector<Eigen::Vector3d> TIC;
extern Eigen::Vector3d G; // gravity

// 零偏估计阈值
extern double BIAS_ACC_THRESHOLD;
extern double BIAS_GYR_THRESHOLD;
extern double SOLVER_TIME;
extern int NUM_ITERATIONS;
extern std::string EX_CALIB_RESULT_PATH;
extern std::string VINS_RESULT_PATH;
extern std::string IMU_TOPIC;

extern double TD; // imu-camera 时间延迟
extern double TR;
extern int ESTIMATE_TD;
extern int ROLLING_SHUTTER;
extern double ROW, COL;

/// @brief 从 ROS 参数服务器读取配置参数
/// @param n
void readParameters(ros::NodeHandle &n);

/// @brief 状态参数化大小
enum SIZE_PARAMETERIZATION {
  SIZE_POSE = 7,      // 位姿（位置+四元数旋转）
  SIZE_SPEEDBIAS = 9, // sudu,加计与陀螺仪零偏
  SIZE_FEATURE = 1    // 特征点逆深度
};

/// @brief 状态变量在状态向量中的顺序
enum StateOrder {
  O_P = 0,  // 平移分量起始索引
  O_R = 3,  // 旋转起始索引
  O_V = 6,  // 速度起始索引
  O_BA = 9, // 加计零偏索引
  O_BG = 12 // 陀螺仪零偏索引
};

/// @brief 噪声项在噪声向量中的顺序
enum NoiseOrder {
  O_AN = 0, // 加计噪声
  O_GN = 3, // 陀螺仪噪声
  O_AW = 6, // 加计随机游走
  O_GW = 9  // 陀螺仪随机游走
};

} // namespace sensor_lab

#endif
