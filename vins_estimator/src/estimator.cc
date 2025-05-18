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
  // 1. check imu observibility
  // 计算滑动窗口所有帧imu加速度均值方差
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

  // 2. 构建SFM特征点数据

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

  // 3. 寻找参考帧并计算相对位姿
  Eigen::Matrix3d relative_R;
  Eigen::Vector3d relative_T;
  int l; // 参考帧索引
  // relativePose 寻找一对具有足够视差的关键帧，计算相对位姿以及参考帧索引l
  // 在滑动窗口中寻找一对具有足够视差的关键帧
  if (!relativePose(relative_R, relative_T, l)) {
    ROS_INFO("Not enough features or parallax; Move device around");
    return false;
  }

  // 4. 全局SFM初始化，计算特征点世界坐标

  // GlobalSFM::construct 完成全局 SFM 初始化，计算特征点空间坐标
  // frame_count + 1：当前帧数；Q, T：输出的帧姿态；l：参考帧索引；
  // relative_R, relative_T：参考帧间的相对位姿
  // sfm_f：特征点观测； sfm_tracked_points ：输出的特征点空间坐标
  // 所有特征点的 3D 坐标存入 sfm_tracked_points
  GlobalSFM sfm;
  if (!sfm.construct(frame_count + 1, Q, T, l, relative_R, relative_T, sfm_f,
                     sfm_tracked_points)) {
    ROS_DEBUG("global SFM failed!");
    marginalization_flag = MARGIN_OLD;
    return false;
  }

  // solve pnp for all frame
  // 为每帧图像计算其在世界坐标系下的相机位姿
  // 对于关键帧，直接使用 SFM 的结果；
  // 对于非关键帧，通过 PnP 求解器结合 SFM 构建的 3D 点云来估计相机位姿
  std::map<double, ImageFrame>::iterator frame_it;
  std::map<int, Eigen::Vector3d>::iterator it;
  // 所有帧的初始姿态 [R, T] 存入 all_image_frame
  frame_it = all_image_frame.begin();
  for (int i = 0; frame_it != all_image_frame.end(); frame_it++) {
    // provide initial guess
    cv::Mat r, rvec, t, D, tmp_r;
    if ((frame_it->first) == Headers[i].stamp.toSec()) {
      // 只有关键帧参与了SFM构造
      frame_it->second.is_key_frame = true;
      // Q[i], T[i] 是 SFM 得到的参考帧下第 i 帧的旋转和平移
      // 再转换到相机坐标系
      frame_it->second.R = Q[i].toRotationMatrix() * RIC[0].transpose();
      frame_it->second.T = T[i];
      i++;
      continue;
    }
    if ((frame_it->first) > Headers[i].stamp.toSec()) {
      i++;
    }

    // 基于 SFM 得到的关键帧位姿 Q[i], T[i]，构造当前非关键帧的初始猜测
    Eigen::Matrix3d R_inital = (Q[i].inverse()).toRotationMatrix();
    Eigen::Vector3d P_inital = -R_inital * T[i];
    cv::eigen2cv(R_inital, tmp_r);
    cv::Rodrigues(tmp_r, rvec);
    cv::eigen2cv(P_inital, t);

    frame_it->second.is_key_frame = false;
    std::vector<cv::Point3f> pts_3_vector;
    std::vector<cv::Point2f> pts_2_vector;
    for (auto &id_pts : frame_it->second.points) {
      int feature_id = id_pts.first;
      for (auto &i_p : id_pts.second) {
        it = sfm_tracked_points.find(feature_id);
        // 收集当前帧中特征点的3d空间坐标和2d坐标
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

    // 5. PnP 为每帧求解相机位姿
    // 对于 SFM 关键帧：直接使用 SFM 解出的位姿
    // 对于非关键帧，使用 SFM 提供的 3D 点云和当前帧的 2D 特征点，使用 OpenCV 的
    // solvePnP 求解相机位姿

    // PNP求解当前帧位姿
    // 假设内参已经归一化
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

  // 6. 视觉与 IMU 对齐
  // 此时滑动窗口内的帧都有合理的初始位姿，可以进行visual-imu initialization
  if (visualInitialAlign())
    return true;
  else {
    ROS_INFO("misalign visual structure with IMU");
    return false;
  }
}

bool Estimator::visualInitialAlign() {
  TicToc t_g;
  Eigen::VectorXd x;
  // solve scale
  // 1. visual-imu alignment 陀螺仪零偏+重力向量优化+尺度因子+对齐到世界坐标系
  bool result = VisualIMUAlignment(all_image_frame, Bgs, g, x);
  if (!result) {
    ROS_DEBUG("solve g failed!");
    return false;
  }

  // change state
  // 2. 更新状态变量为SFM解出的值
  for (int i = 0; i <= frame_count; i++) {
    Eigen::Matrix3d Ri = all_image_frame[Headers[i].stamp.toSec()].R;
    Eigen::Vector3d Pi = all_image_frame[Headers[i].stamp.toSec()].T;
    Ps[i] = Pi;
    Rs[i] = Ri;
    all_image_frame[Headers[i].stamp.toSec()].is_key_frame =
        true; // 所有帧都标为关键帧
  }

  // 3. 清除旧的特征点深度
  Eigen::VectorXd dep = f_manager.getDepthVector();
  for (int i = 0; i < dep.size(); i++)
    dep[i] = -1;
  f_manager.clearDepth(dep);

  // triangulat on cam pose , no tic
  // 4. 三角化特征点，估计特征点深度，仅使用相机位姿，不使用 imu-camera
  // 外参平移向量
  Eigen::Vector3d TIC_TMP[NUM_OF_CAM];
  for (int i = 0; i < NUM_OF_CAM; i++)
    TIC_TMP[i].setZero();

  ric[0] = RIC[0];
  f_manager.setRic(ric);
  f_manager.triangulate(Ps, &(TIC_TMP[0]), &(RIC[0]));

  // 5. 获取尺度因子并基于新的陀螺仪bias重新传播imu data
  double s = (x.tail<1>())(0);
  for (int i = 0; i <= WINDOW_SIZE; i++) {
    // TODO:这里加速度使用0 bias
    pre_integrations[i]->repropagate(Eigen::Vector3d::Zero(), Bgs[i]);
  }

  // 6. 应用尺度因子, 把 SFM 的无尺度位姿转换为真实物理空间中的长度单位
  // 将所有帧的位置统一到imu坐标系下，以第一帧为参考原点
  // Rs[i] * TIC[0] 相机到imu的平移
  // 消除了相机与imu之间的刚体平移
  for (int i = frame_count; i >= 0; i--) {
    // Ps[i] 当前帧的相机位置, Rs[i] 当前帧的相机旋转矩阵, TIC[0] IMU
    // 到相机的外参平移
    Ps[i] = s * Ps[i] - Rs[i] * TIC[0] - (s * Ps[0] - Rs[0] * TIC[0]);
  }

  // 7. 从 x 中提取速度分量，乘以帧旋转矩阵后赋给 Vs[i]
  // 将速度转换为全局坐标系下的表示
  int kv = -1;
  std::map<double, ImageFrame>::iterator frame_i;
  for (frame_i = all_image_frame.begin(); frame_i != all_image_frame.end();
       frame_i++) {
    if (frame_i->second.is_key_frame) {
      kv++;
      Vs[kv] = frame_i->second.R * x.segment<3>(kv * 3);
    }
  }

  // 8. 特征点深度乘以尺度因子 s，使其适应 IMU 坐标系
  for (auto &it_per_id : f_manager.feature) {
    it_per_id.used_num = it_per_id.feature_per_frame.size();
    if (!(it_per_id.used_num >= 2 && it_per_id.start_frame < WINDOW_SIZE - 2))
      continue;
    it_per_id.estimated_depth *= s;
  }

  // 9. 正则化重力方向，yaw 校正
  Eigen::Matrix3d R0 = Utility::g2R(g); // 重力向量转为旋转矩阵
  // Rs[0] 是 SFM 初始化得到的第一帧相机旋转
  // R0 * Rs[0] 将该旋转转换到新的世界坐标系下
  double yaw = Utility::R2ypr(R0 * Rs[0]).x();
  // 用反向yaw 与原 R0 相乘后，相当于对世界坐标系进行旋转，使第一帧的 yaw 变为
  // 0；
  R0 = Utility::ypr2R(Eigen::Vector3d{-yaw, 0, 0}) * R0;
  g = R0 * g; // 新的 g 现在应该指向 [0, 0, -|g|] 方向，并且 yaw=0；

  // 10. 将所有帧的位置、旋转、速度应用旋转校正 rot_diff = R0
  // 确保整个滑动窗口的状态与重力方向一致，消除漂移
  // Matrix3d rot_diff = R0 * Rs[0].transpose();
  Eigen::Matrix3d rot_diff = R0;
  for (int i = 0; i <= frame_count; i++) {
    Ps[i] = rot_diff * Ps[i];
    Rs[i] = rot_diff * Rs[i];
    Vs[i] = rot_diff * Vs[i];
  }
  ROS_DEBUG_STREAM("g0     " << g.transpose());
  ROS_DEBUG_STREAM("my R0  " << Utility::R2ypr(Rs[0]).transpose());

  return true;
}

bool Estimator::relativePose(Eigen::Matrix3d &relative_R,
                             Eigen::Vector3d &relative_T, int &l) {
  // find previous frame which contians enough correspondance and parallex with
  // newest frame 找到与最新帧具有足够对应关系和视差的先前帧
  for (int i = 0; i < WINDOW_SIZE; i++) {
    std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> corres;
    // 从 FeatureManager 中提取帧 i 与最新帧之间的特征点对
    corres = f_manager.getCorresponding(i, WINDOW_SIZE);
    if (corres.size() > 20) {
      double sum_parallax = 0;
      double average_parallax;
      // 估帧 i 与最新帧之间的平均视差
      for (int j = 0; j < int(corres.size()); j++) {
        // pts_0, pts_1 是同一特征点在两帧中的归一化图像坐标
        Eigen::Vector2d pts_0(corres[j].first(0), corres[j].first(1));
        Eigen::Vector2d pts_1(corres[j].second(0), corres[j].second(1));
        double parallax = (pts_0 - pts_1).norm();
        sum_parallax = sum_parallax + parallax;
      }
      average_parallax = 1.0 * sum_parallax / int(corres.size());

      // 判断是否能作为参考帧
      // 对极几何估计帧间RT
      if (average_parallax * FOCAL_LENGTH > 30 &&
          m_estimator.solveRelativeRT(corres, relative_R, relative_T)) {
        l = i;
        ROS_DEBUG("average_parallax %f choose l %d and newest frame to "
                  "triangulate the whole structure",
                  average_parallax * 460, l);
        return true;
      }
    }
  }
  return false;
}

void Estimator::solveOdometry() {
  if (frame_count < WINDOW_SIZE)
    return;
  if (solver_flag == NON_LINEAR) {
    TicToc t_tri;
    // 对f_manager的特征点作三角化，估计特征点深度
    f_manager.triangulate(Ps, tic, ric);
    ROS_DEBUG("triangulation costs %f", t_tri.toc());
    optimization();
  }
}

void Estimator::vector2double() {
  for (int i = 0; i <= WINDOW_SIZE; i++) {
    para_Pose[i][0] = Ps[i].x(); // Ps[i] 是 IMU 坐标系下的位置
    para_Pose[i][1] = Ps[i].y();
    para_Pose[i][2] = Ps[i].z();
    Eigen::Quaterniond q{Rs[i]};
    para_Pose[i][3] = q.x();
    para_Pose[i][4] = q.y();
    para_Pose[i][5] = q.z();
    para_Pose[i][6] = q.w();

    para_SpeedBias[i][0] = Vs[i].x();
    para_SpeedBias[i][1] = Vs[i].y();
    para_SpeedBias[i][2] = Vs[i].z();

    para_SpeedBias[i][3] = Bas[i].x();
    para_SpeedBias[i][4] = Bas[i].y();
    para_SpeedBias[i][5] = Bas[i].z();

    para_SpeedBias[i][6] = Bgs[i].x();
    para_SpeedBias[i][7] = Bgs[i].y();
    para_SpeedBias[i][8] = Bgs[i].z();
  }
  for (int i = 0; i < NUM_OF_CAM; i++) {
    para_Ex_Pose[i][0] = tic[i].x();
    para_Ex_Pose[i][1] = tic[i].y();
    para_Ex_Pose[i][2] = tic[i].z();
    Eigen::Quaterniond q{ric[i]};
    para_Ex_Pose[i][3] = q.x();
    para_Ex_Pose[i][4] = q.y();
    para_Ex_Pose[i][5] = q.z();
    para_Ex_Pose[i][6] = q.w();
  }

  // 当前所有特征点的深度估计值并写入 para_Feature[i][0]
  Eigen::VectorXd dep = f_manager.getDepthVector();
  for (int i = 0; i < f_manager.getFeatureCount(); i++)
    para_Feature[i][0] = dep(i);
  if (ESTIMATE_TD)
    para_Td[0][0] = td;
}

void Estimator::double2vector() {
  Eigen::Vector3d origin_R0 = Utility::R2ypr(Rs[0]);
  Eigen::Vector3d origin_P0 = Ps[0];

  if (failure_occur) {
    origin_R0 = Utility::R2ypr(last_R0);
    origin_P0 = last_P0;
    failure_occur = 0;
  }
  Eigen::Vector3d origin_R00 =
      Utility::R2ypr(Eigen::Quaterniond(para_Pose[0][6], para_Pose[0][3],
                                        para_Pose[0][4], para_Pose[0][5])
                         .toRotationMatrix());
  double y_diff = origin_R0.x() - origin_R00.x();
  // TODO
  Eigen::Matrix3d rot_diff = Utility::ypr2R(Vector3d(y_diff, 0, 0));
  // 处理欧拉角奇异点（pitch 接近 ±90° 的情况）
  if (abs(abs(origin_R0.y()) - 90) < 1.0 ||
      abs(abs(origin_R00.y()) - 90) < 1.0) {
    ROS_DEBUG("euler singular point!");
    rot_diff = Rs[0] * Eigen::Quaterniond(para_Pose[0][6], para_Pose[0][3],
                                          para_Pose[0][4], para_Pose[0][5])
                           .toRotationMatrix()
                           .transpose();
  }

  for (int i = 0; i <= WINDOW_SIZE; i++) {

    Rs[i] = rot_diff * Eigen::Quaterniond(para_Pose[i][6], para_Pose[i][3],
                                          para_Pose[i][4], para_Pose[i][5])
                           .normalized()
                           .toRotationMatrix();

    Ps[i] = rot_diff * Eigen::Vector3d(para_Pose[i][0] - para_Pose[0][0],
                                       para_Pose[i][1] - para_Pose[0][1],
                                       para_Pose[i][2] - para_Pose[0][2]) +
            origin_P0;

    Vs[i] =
        rot_diff * Eigen::Vector3d(para_SpeedBias[i][0], para_SpeedBias[i][1],
                                   para_SpeedBias[i][2]);

    Bas[i] = Eigen::Vector3d(para_SpeedBias[i][3], para_SpeedBias[i][4],
                             para_SpeedBias[i][5]);

    Bgs[i] = Eigen::Vector3d(para_SpeedBias[i][6], para_SpeedBias[i][7],
                             para_SpeedBias[i][8]);
  }

  for (int i = 0; i < NUM_OF_CAM; i++) {
    tic[i] = Eigen::Vector3d(para_Ex_Pose[i][0], para_Ex_Pose[i][1],
                             para_Ex_Pose[i][2]);
    ric[i] = Eigen::Quaterniond(para_Ex_Pose[i][6], para_Ex_Pose[i][3],
                                para_Ex_Pose[i][4], para_Ex_Pose[i][5])
                 .toRotationMatrix();
  }

  Eigen::VectorXd dep = f_manager.getDepthVector();
  for (int i = 0; i < f_manager.getFeatureCount(); i++)
    dep(i) = para_Feature[i][0];
  f_manager.setDepth(dep);
  if (ESTIMATE_TD)
    td = para_Td[0][0];

  // relative info between two loop frame
  // 回环状态更新
  if (relocalization_info) {
    Eigen::Matrix3d relo_r;
    Eigen::Vector3d relo_t;
    relo_r = rot_diff * Eigen::Quaterniond(relo_Pose[6], relo_Pose[3],
                                           relo_Pose[4], relo_Pose[5])
                            .normalized()
                            .toRotationMatrix();
    relo_t = rot_diff * Eigen::Vector3d(relo_Pose[0] - para_Pose[0][0],
                                        relo_Pose[1] - para_Pose[0][1],
                                        relo_Pose[2] - para_Pose[0][2]) +
             origin_P0;
    double drift_correct_yaw;
    drift_correct_yaw =
        Utility::R2ypr(prev_relo_r).x() - Utility::R2ypr(relo_r).x();
    drift_correct_r = Utility::ypr2R(Eigen::Vector3d(drift_correct_yaw, 0, 0));
    drift_correct_t = prev_relo_t - drift_correct_r * relo_t;
    relo_relative_t =
        relo_r.transpose() * (Ps[relo_frame_local_index] - relo_t);
    relo_relative_q = relo_r.transpose() * Rs[relo_frame_local_index];
    relo_relative_yaw =
        Utility::normalizeAngle(Utility::R2ypr(Rs[relo_frame_local_index]).x() -
                                Utility::R2ypr(relo_r).x());
    // cout << "vins relo " << endl;
    // cout << "vins relative_t " << relo_relative_t.transpose() << endl;
    // cout << "vins relative_yaw " <<relo_relative_yaw << endl;
    relocalization_info = 0;
  }
}

bool Estimator::failureDetection() {
  // 当前帧跟踪到的特征点数是否太少
  if (f_manager.last_track_num < 2) {
    ROS_INFO(" little feature %d", f_manager.last_track_num);
    // return true;
  }
  // 最新帧的加计bias模长大小
  if (Bas[WINDOW_SIZE].norm() > 2.5) {
    ROS_INFO(" big IMU acc bias estimation %f", Bas[WINDOW_SIZE].norm());
    return true;
  }
  if (Bgs[WINDOW_SIZE].norm() > 1.0) {
    ROS_INFO(" big IMU gyr bias estimation %f", Bgs[WINDOW_SIZE].norm());
    return true;
  }
  /*
  if (tic(0) > 1)
  {
      ROS_INFO(" big extri param estimation %d", tic(0) > 1);
      return true;
  }
  */
  // 当前帧与上一帧位置差异
  Eigen::Vector3d tmp_P = Ps[WINDOW_SIZE];
  if ((tmp_P - last_P).norm() > 5) {
    ROS_INFO(" big translation");
    return true;
  }
  // 检查z轴平移过大
  if (abs(tmp_P.z() - last_P.z()) > 1) {
    ROS_INFO(" big z translation");
    return true;
  }

  // 当前帧与上一帧之间的相对旋转角度
  Eigen::Matrix3d tmp_R = Rs[WINDOW_SIZE];
  // delta_R 表示两帧间的旋转
  Eigen::Matrix3d delta_R = tmp_R.transpose() * last_R;
  Eigen::Quaterniond delta_Q(delta_R);
  double delta_angle;
  delta_angle = acos(delta_Q.w()) * 2.0 / 3.14 * 180.0;
  if (delta_angle > 50) {
    ROS_INFO(" big delta_angle ");
    // return true;
  }
  return false;
}

void Estimator::setReloFrame(double _frame_stamp, int _frame_index,
                             std::vector<Eigen::Vector3d> &_match_points,
                             Eigen::Vector3d _relo_t, Eigen::Matrix3d _relo_r) {
  // 保存传入的重定位帧时间戳与索引
  relo_frame_stamp = _frame_stamp;
  relo_frame_index = _frame_index;

  // 当前帧与历史帧之间的特征点匹配
  // 用于重定位时构建视觉约束
  match_points.clear();
  match_points = _match_points;

  // 保存上一次重定位帧位姿
  prev_relo_t = _relo_t;
  prev_relo_r = _relo_r;

  // 遍历滑动窗口中的图像帧，查找与 _frame_stamp 对应的帧
  for (int i = 0; i < WINDOW_SIZE; i++) {
    if (relo_frame_stamp == Headers[i].stamp.toSec()) {
      relo_frame_local_index = i;
      relocalization_info = 1;
      for (int j = 0; j < SIZE_POSE; j++)
        relo_Pose[j] = para_Pose[i][j];
    }
  }
}

void Estimator::slideWindow() {
  // 据 marginalization_flag 的值决定是移除旧帧还是保留次新帧
  TicToc t_margin;
  // 边缘化最老帧
  if (marginalization_flag == MARGIN_OLD) {
    double t_0 = Headers[0].stamp.toSec(); // 最旧帧时间戳
    // 保存最旧帧的姿态用于后续残差计算或重定位
    back_R0 = Rs[0];
    back_P0 = Ps[0];
    // 滑动窗口已满
    // 所有状态数组前移一位
    if (frame_count == WINDOW_SIZE) {
      for (int i = 0; i < WINDOW_SIZE; i++) {
        Rs[i].swap(Rs[i + 1]);

        std::swap(pre_integrations[i], pre_integrations[i + 1]);

        dt_buf[i].swap(dt_buf[i + 1]);
        linear_acceleration_buf[i].swap(linear_acceleration_buf[i + 1]);
        angular_velocity_buf[i].swap(angular_velocity_buf[i + 1]);

        Headers[i] = Headers[i + 1];
        Ps[i].swap(Ps[i + 1]);
        Vs[i].swap(Vs[i + 1]);
        Bas[i].swap(Bas[i + 1]);
        Bgs[i].swap(Bgs[i + 1]);
      }
      // 然后用最新的数据覆盖掉最旧帧数据
      Headers[WINDOW_SIZE] = Headers[WINDOW_SIZE - 1];
      Ps[WINDOW_SIZE] = Ps[WINDOW_SIZE - 1];
      Vs[WINDOW_SIZE] = Vs[WINDOW_SIZE - 1];
      Rs[WINDOW_SIZE] = Rs[WINDOW_SIZE - 1];
      Bas[WINDOW_SIZE] = Bas[WINDOW_SIZE - 1];
      Bgs[WINDOW_SIZE] = Bgs[WINDOW_SIZE - 1];

      // 释放并重新初始化最后一个预积分器
      delete pre_integrations[WINDOW_SIZE];
      pre_integrations[WINDOW_SIZE] =
          new IntegrationBase{acc_0, gyr_0, Bas[WINDOW_SIZE], Bgs[WINDOW_SIZE]};

      // 清空最后一个帧的 IMU 缓冲区
      dt_buf[WINDOW_SIZE].clear();
      linear_acceleration_buf[WINDOW_SIZE].clear();
      angular_velocity_buf[WINDOW_SIZE].clear();

      if (true || solver_flag == INITIAL) {
        // 根据时间戳t_0在图像帧缓存中查找
        // 删除该帧以及之前的预计分数据与图像
        std::map<double, ImageFrame>::iterator it_0;
        it_0 = all_image_frame.find(t_0);
        delete it_0->second.pre_integration;
        it_0->second.pre_integration = nullptr;

        for (std::map<double, ImageFrame>::iterator it =
                 all_image_frame.begin();
             it != it_0; ++it) {
          if (it->second.pre_integration)
            delete it->second.pre_integration;
          it->second.pre_integration = NULL;
        }

        all_image_frame.erase(all_image_frame.begin(), it_0);
        all_image_frame.erase(t_0);
      }
      // 边缘化最老帧
      slideWindowOld();
    }
  } else {
    if (frame_count == WINDOW_SIZE) {
      for (unsigned int i = 0; i < dt_buf[frame_count].size(); i++) {
        // 最后一帧的imu数据
        double tmp_dt = dt_buf[frame_count][i];
        Eigen::Vector3d tmp_linear_acceleration =
            linear_acceleration_buf[frame_count][i];
        Eigen::Vector3d tmp_angular_velocity =
            angular_velocity_buf[frame_count][i];

        // 将最后一帧的imu数据追加到倒数第二帧的预计分器中
        pre_integrations[frame_count - 1]->push_back(
            tmp_dt, tmp_linear_acceleration, tmp_angular_velocity);

        dt_buf[frame_count - 1].push_back(tmp_dt);
        linear_acceleration_buf[frame_count - 1].push_back(
            tmp_linear_acceleration);
        angular_velocity_buf[frame_count - 1].push_back(tmp_angular_velocity);
      }

      // 更新倒数第二帧的状态为最后一帧
      Headers[frame_count - 1] = Headers[frame_count];
      Ps[frame_count - 1] = Ps[frame_count];
      Vs[frame_count - 1] = Vs[frame_count];
      Rs[frame_count - 1] = Rs[frame_count];
      Bas[frame_count - 1] = Bas[frame_count];
      Bgs[frame_count - 1] = Bgs[frame_count];

      // 删除并重建最后一帧的预积分器
      delete pre_integrations[WINDOW_SIZE];
      pre_integrations[WINDOW_SIZE] =
          new IntegrationBase{acc_0, gyr_0, Bas[WINDOW_SIZE], Bgs[WINDOW_SIZE]};

      // 清空最后一帧的 IMU 缓冲区
      dt_buf[WINDOW_SIZE].clear();
      linear_acceleration_buf[WINDOW_SIZE].clear();
      angular_velocity_buf[WINDOW_SIZE].clear();

      // 边缘化
      slideWindowNew();
    }
  }
}

void Estimator::slideWindowNew() {
  sum_of_front++;
  // frame_count：当前帧索引，即最新帧
  f_manager.removeFront(
      frame_count); // 移除最新帧的特征点数据，不会改变其他帧状态
}

void Estimator::slideWindowOld() {
  sum_of_back++;

  bool shift_depth =
      solver_flag == NON_LINEAR ? true : false; // 是否处于非线性优化阶段

  // 非线性优化阶段
  if (shift_depth) {
    Eigen::Matrix3d R0, R1; // 旧参考帧（最老帧）的相机位姿
    Eigen::Vector3d P0, P1; // 新参考帧（当前最老帧）的相机位姿

    R0 =
        back_R0 *
        ric[0]; // 上一时刻最老帧（被边缘化的帧）相机的旋转（base-world）=上一时刻最老帧imu*imu-camera
    R1 = Rs[0] * ric[0]; // 当前滑窗中最老帧相机的旋转

    P0 = back_P0 +
         back_R0 *
             tic[0]; // 上一时刻最老帧（被边缘化的帧）相机的位置（base-world）
    P1 = Ps[0] + Rs[0] * tic[0]; // 当前滑窗中最老帧相机的位置

    // 删除旧参考帧（最老帧），并将特征点的深度值从旧参考帧转换到新参考帧下
    // 1. 所有依赖于最老帧观测的特征点，其深度信息需要根据新的参考帧重新计算
    // 2. 保留这些特征点用于后续优化，必须将其深度值转移到新的参考帧下
    f_manager.removeBackShiftDepth(R0, P0, R1, P1);
  } else
    f_manager
        .removeBack(); // 若在初始化阶段，仅删除旧参考帧对应特征点数据，不进行深度调整
}

void Estimator::optimization() {
  ceres::Problem problem;
  ceres::LossFunction *loss_function;
  // Cauchy 损失相比平方误差损失更能容忍大误差的存在
  // 防止异常点对整体优化产生过大影响
  // loss_function = new ceres::HuberLoss(1.0);
  loss_function = new ceres::CauchyLoss(1.0);
  // 定义那些变量需要优化，它们的结构以及使用的损失函数

  // imu状态参数
  for (int i = 0; i < WINDOW_SIZE + 1; i++) {
    ceres::LocalParameterization *local_parameterization =
        new PoseLocalParameterization(); // PoseLocalParameterization()
                                         // 确保旋转在优化过程中保持在流形上
    problem.AddParameterBlock(para_Pose[i], SIZE_POSE,
                              local_parameterization); // SE3 滑窗imu位姿
    problem.AddParameterBlock(
        para_SpeedBias[i],
        SIZE_SPEEDBIAS); // 3个速度 + 3个加速度偏置 + 3个角速度偏置
  }

  // 相机外参
  for (int i = 0; i < NUM_OF_CAM; i++) {
    ceres::LocalParameterization *local_parameterization =
        new PoseLocalParameterization();
    problem.AddParameterBlock(para_Ex_Pose[i], SIZE_POSE,
                              local_parameterization); // SE3

    // 如果 ESTIMATE_EXTRINSIC == false，则para_Ex_Pose被固定不变
    if (!ESTIMATE_EXTRINSIC) {
      ROS_DEBUG("fix extinsic param");
      problem.SetParameterBlockConstant(para_Ex_Pose[i]);
    } else
      ROS_DEBUG("estimate extinsic param");
  }

  // imu-camera time delay
  if (ESTIMATE_TD) {
    problem.AddParameterBlock(para_Td[0], 1);
    // problem.SetParameterBlockConstant(para_Td[0]);
  }

  TicToc t_whole, t_prepare;
  vector2double();

  // 2. 将 IMU 测量值、视觉观测以及先验信息构建成残差项，添加到ceres::Problem
  // 中进行优化

  // a. 先验因子
  // 加入上一次边缘化得到的先验信息，作为对旧状态的约束
  // 如果有，则说明上一轮优化已经进行了边缘化操作
  if (last_marginalization_info) {
    // construct new marginlization_factor
    // 利用上次边缘化的信息构建一个新的残差因子
    MarginalizationFactor *marginalization_factor =
        new MarginalizationFactor(last_marginalization_info);
    // 加入当前优化问题中，使得当前优化仍受历史信息约束
    problem.AddResidualBlock(marginalization_factor, NULL,
                             last_marginalization_parameter_blocks);
  }

  // b. imu因子
  // 用 IMU 预积分数据构建帧间运动模型残差
  for (int i = 0; i < WINDOW_SIZE; i++) {
    int j = i + 1;
    if (pre_integrations[j]->sum_dt > 10.0)
      continue;
    // pre_integrations[j]: 从第 i 帧到第 j 帧的 IMU 预积分结果
    IMUFactor *imu_factor = new IMUFactor(pre_integrations[j]);
    // 更新对姿态 (para_Pose) 和速度/偏差 (para_SpeedBias) 的残差计算
    problem.AddResidualBlock(imu_factor, NULL, para_Pose[i], para_SpeedBias[i],
                             para_Pose[j], para_SpeedBias[j]);

    // 假设滑动窗口状态 [ x0, x1, x2, x3
    // ]，每个xi包含姿态para_Pose[i]，速度与bias para_SpeedBias[i]
    // 对于每对相邻帧 (x0, x1), (x1, x2), (x2, x3)，都会添加一个 IMU 残差项
    // problem.AddResidualBlock(..., para_Pose[0], para_SpeedBias[0],
    // para_Pose[1], para_SpeedBias[1]); problem.AddResidualBlock(...,
    // para_Pose[1], para_SpeedBias[1], para_Pose[2], para_SpeedBias[2]);
    // problem.AddResidualBlock(..., para_Pose[2], para_SpeedBias[2],
    // para_Pose[3], para_SpeedBias[3]); 整个滑动窗口内的 IMU
    // 运动信息都被建模为优化约束
  }

  // c. 构建视觉残差
  int f_m_cnt = 0;
  int feature_index = -1; // 有效特征点索引
  // 遍历特征点
  for (auto &it_per_id : f_manager.feature) {
    it_per_id.used_num = it_per_id.feature_per_frame.size();
    if (!(it_per_id.used_num >= 2 && it_per_id.start_frame < WINDOW_SIZE - 2))
      continue;

    ++feature_index;

    // 当前特征点首次出现的帧索引
    int imu_i = it_per_id.start_frame;
    int imu_j = imu_i - 1;

    // 获取该特征点在起始帧中的归一化图像坐标
    Eigen::Vector3d pts_i = it_per_id.feature_per_frame[0].point;

    // 遍历该特征点在每一帧中的观测记录
    for (auto &it_per_frame : it_per_id.feature_per_frame) {
      imu_j++;
      // 如果是同一帧（即第一个观测），跳过（因为需要两个不同帧之间构造残差）
      if (imu_i == imu_j) {
        continue;
      }

      // 获取j帧下该特征点的归一化图像坐标
      Eigen::Vector3d pts_j = it_per_frame.point;
      // 若启用时间偏移估计则使用带时间延迟矫正的残差ProjectionTdFactor，否则使用ProjectionFactor
      if (ESTIMATE_TD) {
        // 构建残差项
        // pts_i, pts_j: 归一化相机坐标下的特征点方向向量
        // velocity: 特征点在各自帧下的速度
        // cur_td: 时间偏移
        // uv.y(): y 坐标用于去畸变（此处假设已归一化）
        ProjectionTdFactor *f_td = new ProjectionTdFactor(
            pts_i, pts_j, it_per_id.feature_per_frame[0].velocity,
            it_per_frame.velocity, it_per_id.feature_per_frame[0].cur_td,
            it_per_frame.cur_td, it_per_id.feature_per_frame[0].uv.y(),
            it_per_frame.uv.y());
        // 残差项关联的参数块
        // para_Pose[i]: 第 i 帧位姿 [x,y,z,qx,qy,qz,qw]
        // para_Ex_Pose[0]: 相机与 IMU 外参
        // para_Feature[feature_index]: 特征点深度
        // para_Td[0]: 时间偏移 TD
        problem.AddResidualBlock(f_td, loss_function, para_Pose[imu_i],
                                 para_Pose[imu_j], para_Ex_Pose[0],
                                 para_Feature[feature_index], para_Td[0]);
        /*
        double **para = new double *[5];
        para[0] = para_Pose[imu_i];
        para[1] = para_Pose[imu_j];
        para[2] = para_Ex_Pose[0];
        para[3] = para_Feature[feature_index];
        para[4] = para_Td[0];
        f_td->check(para);
        */
      } else {
        ProjectionFactor *f = new ProjectionFactor(pts_i, pts_j);
        problem.AddResidualBlock(f, loss_function, para_Pose[imu_i],
                                 para_Pose[imu_j], para_Ex_Pose[0],
                                 para_Feature[feature_index]);
      }
      f_m_cnt++;
    }
  }

  ROS_DEBUG("visual measurement count: %d", f_m_cnt);
  ROS_DEBUG("prepare for ceres: %f", t_prepare.toc());

  // d. 添加重定位约束
  // 将当前帧与已知的历史关键帧进行对齐
  // 构造视觉重投影残差项，使得系统能够在短暂失跟后重新找回正确的轨迹
  if (relocalization_info) {
    // printf("set relocalization factor! \n");
    ceres::LocalParameterization *local_parameterization =
        new PoseLocalParameterization();
    // relo_Pose 重定位帧位姿
    problem.AddParameterBlock(relo_Pose, SIZE_POSE, local_parameterization);

    int retrive_feature_index = 0;
    int feature_index = -1; // 有效特征点计数器
    for (auto &it_per_id : f_manager.feature) {
      it_per_id.used_num =
          it_per_id.feature_per_frame.size(); // 该特征点被观测到的总帧数
      if (!(it_per_id.used_num >= 2 && it_per_id.start_frame < WINDOW_SIZE - 2))
        continue;

      ++feature_index;
      int start = it_per_id.start_frame; // 该特征点首次出现的帧索引

      // 只处理那些首次出现在重定位帧之前的特征点
      if (start <= relo_frame_local_index) {
        // 直到找到当前特征点 ID 对应的历史匹配点
        while ((int)match_points[retrive_feature_index].z() <
               it_per_id.feature_id) {
          retrive_feature_index++;
        }

        // 找到了对应特征点后继续构造残差项
        if ((int)match_points[retrive_feature_index].z() ==
            it_per_id.feature_id) {
          Eigen::Vector3d pts_j =
              Eigen::Vector3d(match_points[retrive_feature_index].x(),
                              match_points[retrive_feature_index].y(),
                              1.0); // pts_j 该特征点在重定位帧中的位置
          Eigen::Vector3d pts_i =
              it_per_id.feature_per_frame[0]
                  .point; // 该特征点在其首次观测帧中的归一化图像坐标

          // 构建重投影残差
          ProjectionFactor *f = new ProjectionFactor(pts_i, pts_j);
          // 涉及参数：ara_Pose[start]：首次观测该特征点的帧位姿；relo_Pose：重定位帧的位姿；para_Ex_Pose[0]：相机与
          // IMU 的外参；para_Feature[feature_index]：该特征点的深度
          problem.AddResidualBlock(f, loss_function, para_Pose[start],
                                   relo_Pose, para_Ex_Pose[0],
                                   para_Feature[feature_index]);
          retrive_feature_index++;
        }
      }
    }
  }

  ceres::Solver::Options options;

  // 设置线性求解器类型为 Dense
  // Schur，这是适合小规模非线性优化问题的一种高效方法 在 VIO
  // 中，滑动窗口状态量较少，因此使用 Dense Schur 是合理的
  options.linear_solver_type = ceres::DENSE_SCHUR;
  // options.num_threads = 2;
  options.trust_region_strategy_type = ceres::DOGLEG;
  options.max_num_iterations = NUM_ITERATIONS;
  // options.use_explicit_schur_complement = true;
  // options.minimizer_progress_to_stdout = true;
  // options.use_nonmonotonic_steps = true;

  // 如果是边缘化最老帧，则优化的时间略少一些
  if (marginalization_flag == MARGIN_OLD)
    options.max_solver_time_in_seconds = SOLVER_TIME * 4.0 / 5.0;
  else
    options.max_solver_time_in_seconds = SOLVER_TIME;

  TicToc t_solver;
  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);
  // cout << summary.BriefReport() << endl;
  ROS_DEBUG("Iterations : %d", static_cast<int>(summary.iterations.size()));
  ROS_DEBUG("solver costs: %f", t_solver.toc());

  // 将优化后的变量从数组形式转换回 Eigen 向量形式，更新系统状态
  double2vector();

  // e. 边缘化
  TicToc t_whole_marginalization;
  // 当前要边缘化掉滑动窗口中最旧帧
  // 将其对系统状态的影响通过边缘化的方式保留在优化问题中，从而防止信息丢失并维持估计一致性
  if (marginalization_flag == MARGIN_OLD) {
    // marginalization_info 用于记录本次边缘化所需的信息（如残差块、参数块等）
    MarginalizationInfo *marginalization_info = new MarginalizationInfo();
    // 将状态变量从 Eigen 类型转换为数组形式，构建残差项
    vector2double();

    // 如果存在上一次边缘化的信息（即历史先验），则基于它来构建新的边缘化因子
    if (last_marginalization_info) {
      std::vector<int> drop_set; // drop_set：确定哪些参数块需要被边缘化

      for (int i = 0; i < std::static_cast<int>(
                              last_marginalization_parameter_blocks.size());
           i++) {
        // 与最旧帧相关的位姿 para_Pose[0] 和速度/偏差
        // para_SpeedBias[0]需要被边缘化
        if (last_marginalization_parameter_blocks[i] == para_Pose[0] ||
            last_marginalization_parameter_blocks[i] == para_SpeedBias[0])
          drop_set.push_back(i);
      }

      // construct new marginlization_factor
      // 上一次边缘化信息创建一个新的
      // MarginalizationFactor，作为当前优化问题中的先验因子
      MarginalizationFactor *marginalization_factor =
          new MarginalizationFactor(last_marginalization_info);

      // 创建 ResidualBlockInfo，封装残差因子、涉及的参数块和待边缘化参数索引
      ResidualBlockInfo *residual_block_info = new ResidualBlockInfo(
          marginalization_factor, NULL, last_marginalization_parameter_blocks,
          drop_set);

      // 将该残差块加入到当前的 marginalization_info 中，为后续的边缘化做准备
      marginalization_info->addResidualBlockInfo(residual_block_info);
    }

    {
      if (pre_integrations[1]->sum_dt < 10.0) {
        IMUFactor *imu_factor = new IMUFactor(pre_integrations[1]);
        ResidualBlockInfo *residual_block_info = new ResidualBlockInfo(
            imu_factor, NULL,
            std::vector<double *>{para_Pose[0], para_SpeedBias[0], para_Pose[1],
                                  para_SpeedBias[1]},
            std::vector<int>{0, 1});
        marginalization_info->addResidualBlockInfo(residual_block_info);
      }
    }

    {
      int feature_index = -1;
      for (auto &it_per_id : f_manager.feature) {
        it_per_id.used_num = it_per_id.feature_per_frame.size();
        if (!(it_per_id.used_num >= 2 &&
              it_per_id.start_frame < WINDOW_SIZE - 2))
          continue;

        ++feature_index;

        int imu_i = it_per_id.start_frame, imu_j = imu_i - 1;
        if (imu_i != 0)
          continue;

        Eigen::Vector3d pts_i = it_per_id.feature_per_frame[0].point;

        for (auto &it_per_frame : it_per_id.feature_per_frame) {
          imu_j++;
          if (imu_i == imu_j)
            continue;

          Eigen::Vector3d pts_j = it_per_frame.point;
          if (ESTIMATE_TD) {
            ProjectionTdFactor *f_td = new ProjectionTdFactor(
                pts_i, pts_j, it_per_id.feature_per_frame[0].velocity,
                it_per_frame.velocity, it_per_id.feature_per_frame[0].cur_td,
                it_per_frame.cur_td, it_per_id.feature_per_frame[0].uv.y(),
                it_per_frame.uv.y());
            ResidualBlockInfo *residual_block_info = new ResidualBlockInfo(
                f_td, loss_function,
                std::vector<double *>{para_Pose[imu_i], para_Pose[imu_j],
                                      para_Ex_Pose[0],
                                      para_Feature[feature_index], para_Td[0]},
                std::vector<int>{0, 3});
            marginalization_info->addResidualBlockInfo(residual_block_info);
          } else {
            ProjectionFactor *f = new ProjectionFactor(pts_i, pts_j);
            ResidualBlockInfo *residual_block_info = new ResidualBlockInfo(
                f, loss_function,
                std::vector<double *>{para_Pose[imu_i], para_Pose[imu_j],
                                      para_Ex_Pose[0],
                                      para_Feature[feature_index]},
                std::vector<int>{0, 3});
            marginalization_info->addResidualBlockInfo(residual_block_info);
          }
        }
      }
    }

    TicToc t_pre_margin;
    marginalization_info->preMarginalize();
    ROS_DEBUG("pre marginalization %f ms", t_pre_margin.toc());

    TicToc t_margin;
    marginalization_info->marginalize();
    ROS_DEBUG("marginalization %f ms", t_margin.toc());

    std::unordered_map<long, double *> addr_shift;
    for (int i = 1; i <= WINDOW_SIZE; i++) {
      addr_shift[std::reinterpret_cast<long>(para_Pose[i])] = para_Pose[i - 1];
      addr_shift[std::reinterpret_cast<long>(para_SpeedBias[i])] =
          para_SpeedBias[i - 1];
    }
    for (int i = 0; i < NUM_OF_CAM; i++)
      addr_shift[std::reinterpret_cast<long>(para_Ex_Pose[i])] =
          para_Ex_Pose[i];
    if (ESTIMATE_TD) {
      addr_shift[std::reinterpret_cast<long>(para_Td[0])] = para_Td[0];
    }
    std::vector<double *> parameter_blocks =
        marginalization_info->getParameterBlocks(addr_shift);

    if (last_marginalization_info)
      delete last_marginalization_info;
    last_marginalization_info = marginalization_info;
    last_marginalization_parameter_blocks = parameter_blocks;

  } else {
    if (last_marginalization_info &&
        std::count(std::begin(last_marginalization_parameter_blocks),
                   std::end(last_marginalization_parameter_blocks),
                   para_Pose[WINDOW_SIZE - 1])) {

      MarginalizationInfo *marginalization_info = new MarginalizationInfo();
      vector2double();
      if (last_marginalization_info) {
        std::vector<int> drop_set;
        for (int i = 0;
             i < static_cast<int>(last_marginalization_parameter_blocks.size());
             i++) {
          ROS_ASSERT(last_marginalization_parameter_blocks[i] !=
                     para_SpeedBias[WINDOW_SIZE - 1]);
          if (last_marginalization_parameter_blocks[i] ==
              para_Pose[WINDOW_SIZE - 1])
            drop_set.push_back(i);
        }
        // construct new marginlization_factor
        MarginalizationFactor *marginalization_factor =
            new MarginalizationFactor(last_marginalization_info);
        ResidualBlockInfo *residual_block_info = new ResidualBlockInfo(
            marginalization_factor, NULL, last_marginalization_parameter_blocks,
            drop_set);

        marginalization_info->addResidualBlockInfo(residual_block_info);
      }

      TicToc t_pre_margin;
      ROS_DEBUG("begin marginalization");
      marginalization_info->preMarginalize();
      ROS_DEBUG("end pre marginalization, %f ms", t_pre_margin.toc());

      TicToc t_margin;
      ROS_DEBUG("begin marginalization");
      marginalization_info->marginalize();
      ROS_DEBUG("end marginalization, %f ms", t_margin.toc());

      std::unordered_map<long, double *> addr_shift;
      for (int i = 0; i <= WINDOW_SIZE; i++) {
        if (i == WINDOW_SIZE - 1)
          continue;
        else if (i == WINDOW_SIZE) {
          addr_shift[std::reinterpret_cast<long>(para_Pose[i])] =
              para_Pose[i - 1];
          addr_shift[std::reinterpret_cast<long>(para_SpeedBias[i])] =
              para_SpeedBias[i - 1];
        } else {
          addr_shift[std::reinterpret_cast<long>(para_Pose[i])] = para_Pose[i];
          addr_shift[std::reinterpret_cast<long>(para_SpeedBias[i])] =
              para_SpeedBias[i];
        }
      }
      for (int i = 0; i < NUM_OF_CAM; i++)
        addr_shift[std::reinterpret_cast<long>(para_Ex_Pose[i])] =
            para_Ex_Pose[i];
      if (ESTIMATE_TD) {
        addr_shift[std::reinterpret_cast<long>(para_Td[0])] = para_Td[0];
      }

      std::vector<double *> parameter_blocks =
          marginalization_info->getParameterBlocks(addr_shift);
      if (last_marginalization_info)
        delete last_marginalization_info;
      last_marginalization_info = marginalization_info;
      last_marginalization_parameter_blocks = parameter_blocks;
    }
  }
  ROS_DEBUG("whole marginalization costs: %f", t_whole_marginalization.toc());

  ROS_DEBUG("whole time for ceres: %f", t_whole.toc());
}

} // namespace sensor_lab
