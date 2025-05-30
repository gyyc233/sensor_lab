
#include <condition_variable>
#include <cv_bridge/cv_bridge.h>
#include <map>
#include <mutex>
#include <opencv2/opencv.hpp>
#include <queue>
#include <ros/ros.h>
#include <stdio.h>
#include <thread>

#include "estimator.h"
#include "parameters.h"
#include "utility/visualization.h"

using namespace sensor_lab;

// 负责接收ROS传感器数据（IMU、特征点），管理滑动窗口策略，调用 Estimator
// 类进行状态估计，并发布估计结果（位姿、点云、关键帧等） 连接了 IMU
// 和视觉特征输入，驱动 VIO 系统运行

Estimator estimator;

std::condition_variable con; // 条件变量
double current_time = -1;
std::queue<sensor_msgs::ImuConstPtr> imu_buf;            // 缓冲imu
std::queue<sensor_msgs::PointCloudConstPtr> feature_buf; // 缓冲特征点云
std::queue<sensor_msgs::PointCloudConstPtr> relo_buf; // 缓冲重定位点云
int sum_of_wait = 0;

std::mutex m_buf;
std::mutex m_state;
std::mutex i_buf;
std::mutex m_estimator;

double latest_time;
Eigen::Vector3d tmp_P;
Eigen::Quaterniond tmp_Q; // imu 上一时刻的旋转
Eigen::Vector3d tmp_V;
Eigen::Vector3d tmp_Ba; // imu acc bias
Eigen::Vector3d tmp_Bg; // imu gyro bias
Eigen::Vector3d acc_0;  // imu 上一时刻的加速度
Eigen::Vector3d gyr_0;  // imu 上一时刻的角速度
bool init_feature = 0;
bool init_imu = 1;
double last_imu_t = 0; // 上一帧imu时间戳

/// @brief 使用 IMU
/// 数据对当前状态进行预测（前向传播），为非关键帧情况下的位姿提供初始估计
/// @param imu_msg
void predict(const sensor_msgs::ImuConstPtr &imu_msg) {
  double t = imu_msg->header.stamp.toSec();
  if (init_imu) {
    latest_time = t;
    init_imu = 0;
    return;
  }
  double dt = t - latest_time;
  latest_time = t;

  // 提取加速度与角速度数据
  double dx = imu_msg->linear_acceleration.x;
  double dy = imu_msg->linear_acceleration.y;
  double dz = imu_msg->linear_acceleration.z;
  Eigen::Vector3d linear_acceleration{dx, dy, dz};

  double rx = imu_msg->angular_velocity.x;
  double ry = imu_msg->angular_velocity.y;
  double rz = imu_msg->angular_velocity.z;
  Eigen::Vector3d angular_velocity{rx, ry, rz};

  // 上一时刻加速度去偏
  Eigen::Vector3d un_acc_0 = tmp_Q * (acc_0 - tmp_Ba) - estimator.g;

  // 角速度取中值
  Eigen::Vector3d un_gyr = 0.5 * (gyr_0 + angular_velocity) - tmp_Bg;
  // 使用四元数中值法更新当前旋转状态
  // Utility::deltaQ(...) 返回由角速度积分得到的增量旋转
  tmp_Q = tmp_Q * Utility::deltaQ(un_gyr * dt);

  // 当前帧去偏后的线加速度
  Eigen::Vector3d un_acc_1 =
      tmp_Q * (linear_acceleration - tmp_Ba) - estimator.g;

  // 中值加速度
  Eigen::Vector3d un_acc = 0.5 * (un_acc_0 + un_acc_1);

  // 使用中值积分更新位置与速度
  tmp_P = tmp_P + dt * tmp_V + 0.5 * dt * dt * un_acc;
  tmp_V = tmp_V + dt * un_acc;

  acc_0 = linear_acceleration;
  gyr_0 = angular_velocity;
}

void update() {
  TicToc t_predict;
  latest_time = current_time;
  // 从估计器中获取最新状态
  tmp_P = estimator.Ps[WINDOW_SIZE];
  tmp_Q = estimator.Rs[WINDOW_SIZE];
  tmp_V = estimator.Vs[WINDOW_SIZE];
  tmp_Ba = estimator.Bas[WINDOW_SIZE];
  tmp_Bg = estimator.Bgs[WINDOW_SIZE];
  acc_0 = estimator.acc_0;
  gyr_0 = estimator.gyr_0;

  std::queue<sensor_msgs::ImuConstPtr> tmp_imu_buf = imu_buf;
  for (sensor_msgs::ImuConstPtr tmp_imu_msg; !tmp_imu_buf.empty();
       tmp_imu_buf.pop())
    predict(tmp_imu_buf.front());
}

// 每个元素是一个pair: vector<ImuConstPtr>:imu数据列表，PointCloudConstPtr:
// 特征点云数据> 将 IMU 和图像特征数据按时间同步组合，为滑动窗口优化提供输入
std::vector<std::pair<std::vector<sensor_msgs::ImuConstPtr>,
                      sensor_msgs::PointCloudConstPtr>>
getMeasurements() {
  std::vector<std::pair<std::vector<sensor_msgs::ImuConstPtr>,
                        sensor_msgs::PointCloudConstPtr>>
      measurements;

  while (true) {
    if (imu_buf.empty() || feature_buf.empty())
      return measurements;

    if (!(imu_buf.back()->header.stamp.toSec() >
          feature_buf.front()->header.stamp.toSec() + estimator.td)) {
      // ROS_WARN("wait for imu, only should happen at the beginning");
      sum_of_wait++;
      return measurements;
    }

    if (!(imu_buf.front()->header.stamp.toSec() <
          feature_buf.front()->header.stamp.toSec() + estimator.td)) {
      ROS_WARN("throw img, only should happen at the beginning");
      feature_buf.pop();
      continue;
    }

    // 提取一帧图像和对应的 IMU 数据
    sensor_msgs::PointCloudConstPtr img_msg = feature_buf.front();
    feature_buf.pop();

    std::vector<sensor_msgs::ImuConstPtr> IMUs;
    while (imu_buf.front()->header.stamp.toSec() <
           img_msg->header.stamp.toSec() + estimator.td) {
      // 取出所有早于当前图像帧时间的 IMU 数据
      IMUs.emplace_back(imu_buf.front());
      imu_buf.pop();
    }
    //把第一个晚于图像帧时间的 IMU 数据也加入进来,用于后续插值计算
    IMUs.emplace_back(imu_buf.front());
    if (IMUs.empty())
      ROS_WARN("no imu between two image");
    measurements.emplace_back(IMUs, img_msg);
    // 这些数据将在 process() 中传给 estimator.processIMU(...) 和
    // estimator.processImage(...)
  }
  return measurements;
}

void imu_callback(const sensor_msgs::ImuConstPtr &imu_msg) {
  if (imu_msg->header.stamp.toSec() <= last_imu_t) {
    ROS_WARN("imu message in disorder!");
    return;
  }
  last_imu_t = imu_msg->header.stamp.toSec();

  m_buf.lock();
  imu_buf.push(imu_msg);
  m_buf.unlock();
  // 通知等待线程 process() 唤醒数据同步处理流程
  con.notify_one();

  last_imu_t = imu_msg->header.stamp.toSec();

  {
    std::lock_guard<std::mutex> lg(m_state);
    // 使用imu数据对系统状态进行前向传播
    predict(imu_msg);

    std_msgs::Header header = imu_msg->header;
    header.frame_id = "world";
    if (estimator.solver_flag == Estimator::SolverFlag::NON_LINEAR)
      pubLatestOdometry(tmp_P, tmp_Q, tmp_V, header);
  }
}

// 图像特征点数据到达的回调
void feature_callback(const sensor_msgs::PointCloudConstPtr &feature_msg) {
  if (!init_feature) {
    // skip the first detected feature, which doesn't contain optical flow speed
    init_feature = 1;
    return;
  }
  m_buf.lock();
  feature_buf.push(feature_msg);
  m_buf.unlock();
  con.notify_one();
}

void restart_callback(const std_msgs::BoolConstPtr &restart_msg) {
  if (restart_msg->data == true) {
    ROS_WARN("restart the estimator!");
    m_buf.lock();
    while (!feature_buf.empty())
      feature_buf.pop();
    while (!imu_buf.empty())
      imu_buf.pop();
    m_buf.unlock();
    m_estimator.lock();
    estimator.clearState();
    estimator.setParameter();
    m_estimator.unlock();
    current_time = -1;
    last_imu_t = 0;
  }
  return;
}

void relocalization_callback(
    const sensor_msgs::PointCloudConstPtr &points_msg) {
  // printf("relocalization callback! \n");
  m_buf.lock();
  relo_buf.push(points_msg);
  m_buf.unlock();
}

// thread: visual-inertial odometry
void process() {
  while (true) {
    std::vector<std::pair<std::vector<sensor_msgs::ImuConstPtr>,
                          sensor_msgs::PointCloudConstPtr>>
        measurements;
    std::unique_lock<std::mutex> lk(m_buf);

    // getMeasurements()，从 imu_buf 和 feature_buf 中提取时间同步的数据对
    // 如果返回空数据则等待
    con.wait(lk,
             [&] { return (measurements = getMeasurements()).size() != 0; });
    lk.unlock();
    m_estimator.lock();
    for (auto &measurement : measurements) {
      auto img_msg = measurement.second;
      double dx = 0, dy = 0, dz = 0, rx = 0, ry = 0, rz = 0;
      for (auto &imu_msg : measurement.first) {
        double t = imu_msg->header.stamp.toSec();
        double img_t = img_msg->header.stamp.toSec() + estimator.td;
        // 若imu数据早于图像帧则连续积分
        if (t <= img_t) {
          if (current_time < 0)
            current_time = t;
          double dt = t - current_time;
          ROS_ASSERT(dt >= 0);
          current_time = t;
          dx = imu_msg->linear_acceleration.x;
          dy = imu_msg->linear_acceleration.y;
          dz = imu_msg->linear_acceleration.z;
          rx = imu_msg->angular_velocity.x;
          ry = imu_msg->angular_velocity.y;
          rz = imu_msg->angular_velocity.z;
          estimator.processIMU(dt, Vector3d(dx, dy, dz), Vector3d(rx, ry, rz));
          // printf("imu: dt:%f a: %f %f %f w: %f %f %f\n",dt, dx, dy, dz, rx,
          // ry, rz);

        } else {
          // 如果当前 IMU 数据晚于图像帧时间戳,划分两个时间段
          double dt_1 = img_t - current_time;
          double dt_2 = t - img_t;
          current_time = img_t;
          ROS_ASSERT(dt_1 >= 0);
          ROS_ASSERT(dt_2 >= 0);
          ROS_ASSERT(dt_1 + dt_2 > 0);

          // 计算权重
          double w1 = dt_2 / (dt_1 + dt_2);
          double w2 = dt_1 / (dt_1 + dt_2);

          // 双线性插值方法融合前后两段 IMU 数据
          dx = w1 * dx + w2 * imu_msg->linear_acceleration.x;
          dy = w1 * dy + w2 * imu_msg->linear_acceleration.y;
          dz = w1 * dz + w2 * imu_msg->linear_acceleration.z;
          rx = w1 * rx + w2 * imu_msg->angular_velocity.x;
          ry = w1 * ry + w2 * imu_msg->angular_velocity.y;
          rz = w1 * rz + w2 * imu_msg->angular_velocity.z;

          estimator.processIMU(dt_1, Eigen::Vector3d(dx, dy, dz),
                               Eigen::Vector3d(rx, ry, rz));
          // printf("dimu: dt:%f a: %f %f %f w: %f %f %f\n",dt_1, dx, dy, dz,
          // rx, ry, rz);
        }
      }
      // set relocalization frame
      sensor_msgs::PointCloudConstPtr relo_msg = NULL;
      while (!relo_buf.empty()) {
        relo_msg = relo_buf.front();
        relo_buf.pop();
      }
      if (relo_msg != NULL) {
        std::vector<Eigen::Vector3d> match_points;
        double frame_stamp = relo_msg->header.stamp.toSec();
        for (unsigned int i = 0; i < relo_msg->points.size(); i++) {
          Eigen::Vector3d u_v_id;
          u_v_id.x() = relo_msg->points[i].x;
          u_v_id.y() = relo_msg->points[i].y;
          u_v_id.z() = relo_msg->points[i].z;
          match_points.push_back(u_v_id);
        }
        Eigen::Vector3d relo_t(relo_msg->channels[0].values[0],
                               relo_msg->channels[0].values[1],
                               relo_msg->channels[0].values[2]);
        Quaterniond relo_q(
            relo_msg->channels[0].values[3], relo_msg->channels[0].values[4],
            relo_msg->channels[0].values[5], relo_msg->channels[0].values[6]);
        Eigen::Matrix3d relo_r = relo_q.toRotationMatrix();
        int frame_index;
        frame_index = relo_msg->channels[0].values[7];
        estimator.setReloFrame(frame_stamp, frame_index, match_points, relo_t,
                               relo_r);
      }

      ROS_DEBUG("processing vision data with stamp %f \n",
                img_msg->header.stamp.toSec());

      TicToc t_s;
      std::map<int, std::vector<std::pair<int, Eigen::Matrix<double, 7, 1>>>>
          image;
      for (unsigned int i = 0; i < img_msg->points.size(); i++) {
        int v = img_msg->channels[0].values[i] + 0.5;
        int feature_id = v / NUM_OF_CAM;
        int camera_id = v % NUM_OF_CAM;
        double x = img_msg->points[i].x;
        double y = img_msg->points[i].y;
        double z = img_msg->points[i].z;
        double p_u = img_msg->channels[1].values[i];
        double p_v = img_msg->channels[2].values[i];
        double velocity_x = img_msg->channels[3].values[i];
        double velocity_y = img_msg->channels[4].values[i];
        ROS_ASSERT(z == 1);
        Eigen::Matrix<double, 7, 1> xyz_uv_velocity;
        xyz_uv_velocity << x, y, z, p_u, p_v, velocity_x, velocity_y;
        image[feature_id].emplace_back(camera_id, xyz_uv_velocity);
      }
      estimator.processImage(image, img_msg->header);

      double whole_t = t_s.toc();
      printStatistics(estimator, whole_t);
      std_msgs::Header header = img_msg->header;
      header.frame_id = "world";

      pubOdometry(estimator, header);
      pubKeyPoses(estimator, header);
      pubCameraPose(estimator, header);
      pubPointCloud(estimator, header);
      pubTF(estimator, header);
      pubKeyframe(estimator);
      if (relo_msg != NULL)
        pubRelocalization(estimator);
      // ROS_ERROR("end: %f, at %f", img_msg->header.stamp.toSec(),
      // ros::Time::now().toSec());
    }
    m_estimator.unlock();
    m_buf.lock();
    m_state.lock();
    if (estimator.solver_flag == Estimator::SolverFlag::NON_LINEAR)
      update();
    m_state.unlock();
    m_buf.unlock();
  }
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "vins_estimator");
  ros::NodeHandle n("~");
  ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME,
                                 ros::console::levels::Info);
  readParameters(n);
  estimator.setParameter();
#ifdef EIGEN_DONT_PARALLELIZE
  ROS_DEBUG("EIGEN_DONT_PARALLELIZE");
#endif
  ROS_WARN("waiting for image and imu...");

  registerPub(n);

  ros::Subscriber sub_imu = n.subscribe(IMU_TOPIC, 2000, imu_callback,
                                        ros::TransportHints().tcpNoDelay());
  ros::Subscriber sub_image =
      n.subscribe("/feature_tracker/feature", 2000, feature_callback);
  ros::Subscriber sub_restart =
      n.subscribe("/feature_tracker/restart", 2000, restart_callback);
  ros::Subscriber sub_relo_points =
      n.subscribe("/pose_graph/match_points", 2000, relocalization_callback);

  std::thread measurement_process{process};
  ros::spin();

  return 0;
}
