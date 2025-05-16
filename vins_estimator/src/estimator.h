#ifndef VINS_ESTIMATOR_ESTIMATOR
#define VINS_ESTIMATOR_ESTIMATOR

#include "feature_manager.h"
#include "initial/initial_alignment.h"
#include "initial/initial_ex_rotation.h"
#include "initial/initial_sfm.h"
#include "initial/solve_5pts.h"
#include "parameters.h"
#include "utility/tic_toc.h"
#include "utility/utility.h"
#include <std_msgs/Float32.h>
#include <std_msgs/Header.h>

#include "factor/imu_factor.h"
#include "factor/marginalization_factor.h"
#include "factor/pose_local_parameterization.h"
#include "factor/projection_factor.h"
#include "factor/projection_td_factor.h"
#include <ceres/ceres.h>

#include <opencv2/core/eigen.hpp>
#include <queue>
#include <unordered_map>

// 整个 VIO（Visual-Inertial Odometry）系统的核心类
// 负责融合 IMU 和视觉信息，进行状态估计、初始化、滑动窗口优化、边缘化等操作

namespace sensor_lab {

class Estimator {
public:
  Estimator();

  /// @brief 设置相机参数，imu噪声模型等
  void setParameter();

  /// @brief imu 预积分
  /// @param t
  /// @param linear_acceleration
  /// @param angular_velocity
  void processIMU(double t, const Eigen::Vector3d &linear_acceleration,
                  const Eigen::Vector3d &angular_velocity);

  /// @brief 调用 FeatureManager 管理图像特征点生命周期
  /// @param image
  /// @param header
  void processImage(
      const std::map<
          int, std::vector<std::pair<int, Eigen::Matrix<double, 7, 1>>>> &image,
      const std_msgs::Header &header);

  /// @brief 设置重定位帧与匹配点，用于闭环或漂移矫正
  /// @note
  /// 设置重定位帧的信息，以便在系统发生故障或需要回环检测时，能够基于已知地图信息进行相对位姿估计和漂移校正
  /// @param _frame_stamp 重定位帧时间戳
  /// @param _frame_index 重定位帧索引
  /// @param _match_points 当前帧与历史帧之间的特征点匹配
  /// @param _relo_t
  /// @param _relo_r
  void setReloFrame(double _frame_stamp, int _frame_index,
                    std::vector<Eigen::Vector3d> &_match_points,
                    Eigen::Vector3d _relo_t, Eigen::Matrix3d _relo_r);

  // internal
  void clearState();

  /// @brief 根据滑动窗口中的多帧图像恢复结构（SfM 初始化）
  /// @return
  bool initialStructure();

  /// @brief camera-imu 对齐，进行尺度恢复
  /// @return
  bool visualInitialAlign();

  /// @brief 在滑动窗口中寻找一个参考帧（记为
  /// i），该帧与最新帧（WINDOW_SIZE）之间具有足够的特征点对应关系和视差（parallax）
  /// @param relative_R
  /// @param relative_T
  /// @param l 参考帧索引
  /// @return
  bool relativePose(Eigen::Matrix3d &relative_R, Eigen::Vector3d &relative_T,
                    int &l);

  // 滑动窗口更新
  void slideWindow();
  void slideWindowNew();
  void slideWindowOld();

  // 非线性优化
  /// @brief 求解里程计
  void solveOdometry();
  void optimization();

  // 将状态变量从 Eigen 向量转为 Ceres 可优化的 double 数组
  void vector2double();
  void double2vector();

  // 故障检测，如轨迹跳跃、深度无效等，出现问题触发reset
  bool failureDetection();

  enum SolverFlag { INITIAL, NON_LINEAR };

  enum MarginalizationFlag { MARGIN_OLD = 0, MARGIN_SECOND_NEW = 1 };

  SolverFlag solver_flag;                   // 是否完成初始化
  MarginalizationFlag marginalization_flag; // 边缘化方式
  Eigen::Vector3d g;                        // 估计的重力向量
  Eigen::MatrixXd Ap[2], backup_A;
  Eigen::VectorXd bp[2], backup_b;

  // camera to imu extrinsic parameters
  Eigen::Matrix3d ric[NUM_OF_CAM];
  Eigen::Vector3d tic[NUM_OF_CAM];

  // 滑动窗口对应变量包含 WINDOW_SIZE
  // 系统状态变量，每个图像帧对应的imu位姿，初始值由imu预积分传播的来
  Eigen::Vector3d Ps[(WINDOW_SIZE + 1)];
  Eigen::Vector3d Vs[(WINDOW_SIZE + 1)];
  Eigen::Matrix3d Rs[(WINDOW_SIZE + 1)];
  Eigen::Vector3d Bas[(WINDOW_SIZE + 1)];
  Eigen::Vector3d Bgs[(WINDOW_SIZE + 1)];
  double td; // imu-camera time delay

  Eigen::Matrix3d back_R0, last_R, last_R0;
  Eigen::Vector3d back_P0, last_P, last_P0;
  std_msgs::Header Headers[(WINDOW_SIZE + 1)];

  // 每个图像帧对应的imu预积分结果
  IntegrationBase *pre_integrations[(WINDOW_SIZE + 1)];
  Eigen::Vector3d acc_0; // 上一时刻线加速度
  Eigen::Vector3d gyr_0; // 上一时刻角速度

  std::vector<double> dt_buf[(WINDOW_SIZE + 1)];
  std::vector<Eigen::Vector3d> linear_acceleration_buf[(WINDOW_SIZE + 1)];
  std::vector<Eigen::Vector3d> angular_velocity_buf[(WINDOW_SIZE + 1)];

  int frame_count; // 帧计数器
  int sum_of_outlier, sum_of_back, sum_of_front, sum_of_invalid;

  // 管理所有特征点的生命周期和观测信息
  // 包括每个特征点在哪些帧中被观测到、深度估计、是否失效
  // 在滑动窗口优化中参与视觉重投影误差的构建
  FeatureManager f_manager;
  MotionEstimator m_estimator;
  InitialEXRotation initial_ex_rotation;

  bool first_imu; // 首次收到imu数据
  bool is_valid, is_key;
  bool failure_occur;

  std::vector<Eigen::Vector3d> point_cloud;  // 当前地图点
  std::vector<Eigen::Vector3d> margin_cloud; // 被边缘化的地图点
  std::vector<Eigen::Vector3d> key_poses;    // 关键帧位姿
  double initial_timestamp;

  // 优化参数块
  double para_Pose[WINDOW_SIZE + 1][SIZE_POSE]; // [x,y,z,qx,qy,qz,qw]
  double para_SpeedBias
      [WINDOW_SIZE + 1]
      [SIZE_SPEEDBIAS]; // [v_x,v_y,v_z,b_a_x,b_a_y,b_a_z,b_g_x,b_g_y,b_g_z]
  double para_Feature[NUM_OF_F][SIZE_FEATURE]; // 特征点逆深度或真实深度
  double para_Ex_Pose[NUM_OF_CAM]
                     [SIZE_POSE]; // camera-imu extrinsic translation [tic, ric]
  double para_Retrive_Pose[SIZE_POSE];
  double para_Td[1][1]; // imu-camera 时间同步偏差
  double para_Tr[1][1];

  int loop_window_index;

  MarginalizationInfo *last_marginalization_info; // 上一次边缘化信息
  std::vector<double *> last_marginalization_parameter_blocks;

  // 图像帧缓存 key为时间戳, value 为 ImageFrame
  // ImageFrame 包含 a. 特征点信息；b. 对应imu预积分器；c. 是否是关键帧；d.
  // 当前帧相对参考帧的RT; e. 图像时间戳等
  std::map<double, ImageFrame> all_image_frame;
  IntegrationBase *tmp_pre_integration;

  // relocalization variable 重定位相关
  bool relocalization_info;
  double relo_frame_stamp;
  double relo_frame_index;
  int relo_frame_local_index;
  std::vector<Eigen::Vector3d> match_points;
  double relo_Pose[SIZE_POSE];

  Eigen::Matrix3d drift_correct_r;
  Eigen::Vector3d drift_correct_t;
  Eigen::Vector3d prev_relo_t;
  Eigen::Matrix3d prev_relo_r;
  Eigen::Vector3d relo_relative_t;
  Eigen::Quaterniond relo_relative_q;
  double relo_relative_yaw;
} // namespace sensor_lab

#endif
