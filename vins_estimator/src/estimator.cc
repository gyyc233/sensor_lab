#include "estimator.h"

namespace sensor_lab {
Estimator::Estimator() : f_manager{Rs} {
  ROS_INFO("init begins");
  clearState();
}

void Estimator::setParameter() {
  for (int i = 0; i < NUM_OF_CAM; i++) {
    tic[i] = TIC[i];
    ric[i] = RIC[i];
  }
  f_manager.setRic(ric);
  // 设置信息矩阵
  // FOCAL_LENGTH / 1.5 影响优化过程中视觉残差的置信度
  // 数值越小，表示对视觉观测越信任；数值越大，则认为视觉误差较大
  ProjectionFactor::sqrt_info =
      FOCAL_LENGTH / 1.5 * Eigen::Matrix2d::Identity();
  ProjectionTdFactor::sqrt_info =
      FOCAL_LENGTH / 1.5 * Eigen::Matrix2d::Identity();
  td = TD;
}

void Estimator::clearState() {
  for (int i = 0; i < WINDOW_SIZE + 1; i++) {
    Rs[i].setIdentity();
    Ps[i].setZero();
    Vs[i].setZero();
    Bas[i].setZero();
    Bgs[i].setZero();
    dt_buf[i].clear();
    linear_acceleration_buf[i].clear();
    angular_velocity_buf[i].clear();

    // 清除imu预积分数据
    if (pre_integrations[i] != nullptr)
      delete pre_integrations[i];
    pre_integrations[i] = nullptr;
  }

  for (int i = 0; i < NUM_OF_CAM; i++) {
    tic[i] = Eigen::Vector3d::Zero();
    ric[i] = Eigen::Matrix3d::Identity();
  }

  // 清除图像帧缓存中的预积分信息
  for (auto &it : all_image_frame) {
    // 每个图像帧包含对应的 IMU 预积分器
    if (it.second.pre_integration != nullptr) {
      delete it.second.pre_integration;
      it.second.pre_integration = nullptr;
    }
  }

  solver_flag = INITIAL; // 表示系统处于初始化阶段
  first_imu = false, sum_of_back = 0;
  sum_of_front = 0;
  frame_count = 0;
  initial_timestamp = 0;
  all_image_frame.clear();
  td = TD;

  if (tmp_pre_integration != nullptr)
    delete tmp_pre_integration;
  if (last_marginalization_info != nullptr)
    delete last_marginalization_info;

  tmp_pre_integration = nullptr;
  last_marginalization_info = nullptr;
  last_marginalization_parameter_blocks.clear();

  // 清除所有特征点观测和生命周期信息
  f_manager.clearState();

  // 重置故障检测与重定位标志
  failure_occur = 0;
  relocalization_info = 0;

  drift_correct_r = Eigen::Matrix3d::Identity();
  drift_correct_t = Eigen::Vector3d::Zero();
}

void Estimator::processIMU(double dt,
                           const Eigen::Vector3d &linear_acceleration,
                           const Eigen::Vector3d &angular_velocity) {
  if (!first_imu) {
    first_imu = true;
    acc_0 = linear_acceleration;
    gyr_0 = angular_velocity;
  }

  if (!pre_integrations[frame_count]) {
    // 每帧图像对应一个预积分器
    pre_integrations[frame_count] =
        new IntegrationBase{acc_0, gyr_0, Bas[frame_count], Bgs[frame_count]};
  }

  // 非首帧，进行状态传播
  if (frame_count != 0) {
    // 添加imu数据到当前图像帧对应预积分器
    pre_integrations[frame_count]->push_back(dt, linear_acceleration,
                                             angular_velocity);
    // if(solver_flag != NON_LINEAR)
    // 在初始化阶段用于粗略状态传播
    tmp_pre_integration->push_back(dt, linear_acceleration, angular_velocity);

    // 缓存当前帧imu原始数据
    dt_buf[frame_count].push_back(dt);
    linear_acceleration_buf[frame_count].push_back(linear_acceleration);
    angular_velocity_buf[frame_count].push_back(angular_velocity);

    // 使用 IMU 测量值对当前帧的姿态、位置和速度进行预测（前向传播）
    int j = frame_count;
    Eigen::Vector3d un_acc_0 = Rs[j] * (acc_0 - Bas[j]) - g;
    Eigen::Vector3d un_gyr =
        0.5 * (gyr_0 + angular_velocity) - Bgs[j]; // 去偏后的角速度
    // 角速度积分
    Rs[j] *= Utility::deltaQ(un_gyr * dt).toRotationMatrix();
    Eigen::Vector3d un_acc_1 = Rs[j] * (linear_acceleration - Bas[j]) - g;

    // 中值加速度
    Eigen::Vector3d un_acc = 0.5 * (un_acc_0 + un_acc_1);
    // 位置 速度更新
    Ps[j] += dt * Vs[j] + 0.5 * dt * dt * un_acc;
    Vs[j] += dt * un_acc;
  }

  acc_0 = linear_acceleration;
  gyr_0 = angular_velocity;
}

void Estimator::processImage(
    const map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>> &image,
    const std_msgs::Header &header) {
  ROS_DEBUG("new image coming ------------------------------------------");
  ROS_DEBUG("Adding feature points %lu", image.size());
  // 检测当前帧的特征点并检查视差
  if (f_manager.addFeatureCheckParallax(frame_count, image, td)) {
    // 视差足够大，认为是关键帧，设置滑动窗口策略
    marginalization_flag = MARGIN_OLD;
  } else {
    // 不是关键帧
    marginalization_flag = MARGIN_SECOND_NEW;
  }

  ROS_DEBUG("this frame is--------------------%s",
            marginalization_flag ? "reject" : "accept");
  ROS_DEBUG("%s", marginalization_flag ? "Non-keyframe" : "Keyframe");
  ROS_DEBUG("Solving %d", frame_count);
  ROS_DEBUG("number of feature: %d", f_manager.getFeatureCount());
  Headers[frame_count] = header;

  // 构建图像帧结构并插入缓存
  ImageFrame imageframe(image, header.stamp.toSec());
  imageframe.pre_integration = tmp_pre_integration;
  all_image_frame.insert(make_pair(header.stamp.toSec(), imageframe));
  tmp_pre_integration =
      new IntegrationBase{acc_0, gyr_0, Bas[frame_count], Bgs[frame_count]};

  // 需要在线外参旋转矩阵标定
  if (ESTIMATE_EXTRINSIC == 2) {
    ROS_INFO("calibrating extrinsic param, rotation movement is needed");
    if (frame_count != 0) {
      // 使用两帧之间的对应点和预积分旋转估计来估计 camera-imu rotation matrix
      std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> corres =
          f_manager.getCorresponding(frame_count - 1, frame_count);
      Eigen::Matrix3d calib_ric;
      if (initial_ex_rotation.CalibrationExRotation(
              corres, pre_integrations[frame_count]->delta_q, calib_ric)) {
        ROS_WARN("initial extrinsic rotation calib success");
        ROS_WARN_STREAM("initial extrinsic rotation: " << endl << calib_ric);
        ric[0] = calib_ric;
        RIC[0] = calib_ric;
        ESTIMATE_EXTRINSIC = 1;
      }
    }
  }

  // 若系统处于初始化阶段
  if (solver_flag == INITIAL) {
    // 当帧数达到滑动窗口大小时初始化结构
    if (frame_count == WINDOW_SIZE) {
      bool result = false;
      if (ESTIMATE_EXTRINSIC != 2 &&
          (header.stamp.toSec() - initial_timestamp) > 0.1) {
        // SFM（Structure from Motion）初始化
        result = initialStructure();
        initial_timestamp = header.stamp.toSec();
      }
      // 初始化成功
      if (result) {
        solver_flag = NON_LINEAR; // 切换为非线性优化模式
        solveOdometry();
        slideWindow();              // 滑动窗口更新
        f_manager.removeFailures(); // 移除失败特征点
        ROS_INFO("Initialization finish!");

        // 更新位姿
        last_R = Rs[WINDOW_SIZE];
        last_P = Ps[WINDOW_SIZE];
        last_R0 = Rs[0];
        last_P0 = Ps[0];

      } else
        slideWindow(); //失败则仅滑动窗口
    } else
      frame_count++;
  }
  // 非线性优化过程
  else {
    TicToc t_solve;
    solveOdometry();
    ROS_DEBUG("solver costs: %fms", t_solve.toc());

    // 若检测跟踪失败则重启
    if (failureDetection()) {
      ROS_WARN("failure detection!");
      failure_occur = 1;
      clearState();
      setParameter();
      ROS_WARN("system reboot!");
      return;
    }

    TicToc t_margin;
    slideWindow();
    f_manager.removeFailures();
    ROS_DEBUG("marginalization costs: %fms", t_margin.toc());
    // prepare output of VINS
    // 记录所有关键帧位姿
    key_poses.clear();
    for (int i = 0; i <= WINDOW_SIZE; i++)
      key_poses.push_back(Ps[i]);

    // update state values
    last_R = Rs[WINDOW_SIZE];
    last_P = Ps[WINDOW_SIZE];
    last_R0 = Rs[0];
    last_P0 = Ps[0];
  }
}
bool Estimator::initialStructure() {
  TicToc t_sfm;
  // check imu observibility
  {
    std::map<double, ImageFrame>::iterator frame_it;
    // 计算所有图像帧对应的 IMU 加速度均值
    Eigen::Vector3d sum_g;
    for (frame_it = all_image_frame.begin(), frame_it++;
         frame_it != all_image_frame.end(); frame_it++) {
      double dt = frame_it->second.pre_integration->sum_dt;
      Eigen::Vector3d tmp_g =
          frame_it->second.pre_integration->delta_v / dt; // 平均加速度
      sum_g += tmp_g; // 累加得到总的重力方向加速度 sum_g
    }

    // 计算平均加速度 aver_g，作为估计重力方向的基础
    Eigen::Vector3d aver_g;
    aver_g = sum_g * 1.0 / ((int)all_image_frame.size() - 1);

    double var = 0;
    for (frame_it = all_image_frame.begin(), frame_it++;
         frame_it != all_image_frame.end(); frame_it++) {
      double dt = frame_it->second.pre_integration->sum_dt;
      Eigen::Vector3d tmp_g = frame_it->second.pre_integration->delta_v / dt;
      var += (tmp_g - aver_g).transpose() * (tmp_g - aver_g);
      // cout << "frame g " << tmp_g.transpose() << endl;
    }
    // 计算加速度变化的方差 var，用于判断设备是否移动充分
    var = sqrt(var / ((int)all_image_frame.size() - 1));
    // ROS_WARN("IMU variation %f!", var);
    if (var < 0.25) {
      ROS_INFO("IMU excitation not enouth!");
      // return false;
    }
  }

  // global sfm
  // 保存每一帧的旋转和平移
  Eigen::Quaterniond Q[frame_count + 1];
  Eigen::Vector3d T[frame_count + 1];
  // 保存世界坐标系下特征点
  std::map<int, Eigen::Vector3d> sfm_tracked_points;
  std::vector<SFMFeature> sfm_f;
  for (auto &it_per_id : f_manager.feature) {
    int imu_j = it_per_id.start_frame - 1;
    SFMFeature tmp_feature;
    tmp_feature.state = false;
    tmp_feature.id = it_per_id.feature_id;
    // 记录特征点在每一帧下的2d像素坐标 tmp_feature
    for (auto &it_per_frame : it_per_id.feature_per_frame) {
      imu_j++;
      Eigen::Vector3d pts_j = it_per_frame.point;
      tmp_feature.observation.push_back(
          make_pair(imu_j, Eigen::Vector2d{pts_j.x(), pts_j.y()}));
    }
    sfm_f.push_back(tmp_feature);
  }

  Eigen::Matrix3d relative_R;
  Eigen::Vector3d relative_T;
  int l;
  // relativePose 寻找一对具有足够视差的关键帧，计算相对位姿以及参考帧索引l
  if (!relativePose(relative_R, relative_T, l)) {
    ROS_INFO("Not enough features or parallax; Move device around");
    return false;
  }

  // GlobalSFM::construct 完成全局 SFM 初始化，计算特征点空间坐标
  // frame_count + 1：当前帧数；Q, T：输出的帧姿态；l：参考帧索引；
  // relative_R, relative_T：参考帧间的相对位姿
  // sfm_f：特征点观测；sfm_tracked_points：输出的特征点空间坐标
  GlobalSFM sfm;
  if (!sfm.construct(frame_count + 1, Q, T, l, relative_R, relative_T, sfm_f,
                     sfm_tracked_points)) {
    ROS_DEBUG("global SFM failed!");
    marginalization_flag = MARGIN_OLD;
    return false;
  }

  // solve pnp for all frame
  std::map<double, ImageFrame>::iterator frame_it;
  std::map<int, Eigen::Vector3d>::iterator it;
  frame_it = all_image_frame.begin();
  for (int i = 0; frame_it != all_image_frame.end(); frame_it++) {
    // provide initial guess
    cv::Mat r, rvec, t, D, tmp_r;
    if ((frame_it->first) == Headers[i].stamp.toSec()) {
      frame_it->second.is_key_frame = true;
      frame_it->second.R = Q[i].toRotationMatrix() * RIC[0].transpose();
      frame_it->second.T = T[i];
      i++;
      continue;
    }
    if ((frame_it->first) > Headers[i].stamp.toSec()) {
      i++;
    }
    Eigen::Matrix3d R_inital = (Q[i].inverse()).toRotationMatrix();
    Eigen::Vector3d P_inital = -R_inital * T[i];
    cv::eigen2cv(R_inital, tmp_r);
    cv::Rodrigues(tmp_r, rvec);
    cv::eigen2cv(P_inital, t);

    frame_it->second.is_key_frame = false;
    vector<cv::Point3f> pts_3_vector;
    vector<cv::Point2f> pts_2_vector;
    for (auto &id_pts : frame_it->second.points) {
      int feature_id = id_pts.first;
      for (auto &i_p : id_pts.second) {
        it = sfm_tracked_points.find(feature_id);
        if (it != sfm_tracked_points.end()) {
          Eigen::Vector3d world_pts = it->second;
          cv::Point3f pts_3(world_pts(0), world_pts(1), world_pts(2));
          pts_3_vector.push_back(pts_3);
          Eigen::Vector2d img_pts = i_p.second.head<2>();
          cv::Point2f pts_2(img_pts(0), img_pts(1));
          pts_2_vector.push_back(pts_2);
        }
      }
    }
    cv::Mat K = (cv::Mat_<double>(3, 3) << 1, 0, 0, 0, 1, 0, 0, 0, 1);
    if (pts_3_vector.size() < 6) {
      cout << "pts_3_vector size " << pts_3_vector.size() << endl;
      ROS_DEBUG("Not enough points for solve pnp !");
      return false;
    }
    if (!cv::solvePnP(pts_3_vector, pts_2_vector, K, D, rvec, t, 1)) {
      ROS_DEBUG("solve pnp fail!");
      return false;
    }
    cv::Rodrigues(rvec, r);
    Eigen::MatrixXd R_pnp, tmp_R_pnp;
    cv::cv2eigen(r, tmp_R_pnp);
    R_pnp = tmp_R_pnp.transpose();
    Eigen::MatrixXd T_pnp;
    cv::cv2eigen(t, T_pnp);
    T_pnp = R_pnp * (-T_pnp);
    frame_it->second.R = R_pnp * RIC[0].transpose();
    frame_it->second.T = T_pnp;
  }
  if (visualInitialAlign())
    return true;
  else {
    ROS_INFO("misalign visual structure with IMU");
    return false;
  }
}
} // namespace sensor_lab
