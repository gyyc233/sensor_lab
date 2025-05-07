#include "initial_sfm.h"

namespace sensor_lab {
GlobalSFM::GlobalSFM() {}

GlobalSFM::~GlobalSFM() {}

void GlobalSFM::triangulatePoint(Eigen::Matrix<double, 3, 4> &Pose0,
                                 Eigen::Matrix<double, 3, 4> &Pose1,
                                 Eigen::Vector2d &point0,
                                 Eigen::Vector2d &point1,
                                 Eigen::Vector3d &point_3d) {
  // DLT方法 SVD分解求三角化
  // https://blog.csdn.net/Walking_roll/article/details/119984469
  // 使用同向向量叉积为0构造齐次方程
  // 与14讲计算三角化的原理不同
  Matrix4d design_matrix = Matrix4d::Zero();
  design_matrix.row(0) = point0[0] * Pose0.row(2) - Pose0.row(0);
  design_matrix.row(1) = point0[1] * Pose0.row(2) - Pose0.row(1);
  design_matrix.row(2) = point1[0] * Pose1.row(2) - Pose1.row(0);
  design_matrix.row(3) = point1[1] * Pose1.row(2) - Pose1.row(1);

  Vector4d triangulated_point;
  triangulated_point =
      design_matrix.jacobiSvd(Eigen::ComputeFullV).matrixV().rightCols<1>();

  // 将齐次坐标 (x, y, z, w) 转换为三维空间坐标 (x/w, y/w, z/w)
  point_3d(0) = triangulated_point(0) / triangulated_point(3);
  point_3d(1) = triangulated_point(1) / triangulated_point(3);
  point_3d(2) = triangulated_point(2) / triangulated_point(3);
}

void GlobalSFM::triangulateTwoFrames(int frame0,
                                     Eigen::Matrix<double, 3, 4> &Pose0,
                                     int frame1,
                                     Eigen::Matrix<double, 3, 4> &Pose1,
                                     std::vector<SFMFeature> &sfm_f) {
  assert(frame0 != frame1);

  // 遍历所有特征点
  for (int j = 0; j < feature_num; j++) {
    // 若该特征点已完成三角化则跳过
    if (sfm_f[j].state == true)
      continue;

    bool has_0 = false, has_1 = false;
    Eigen::Vector2d point0;
    Eigen::Vector2d point1;
    // 遍历该特征点的所有观测帧
    for (int k = 0; k < (int)sfm_f[j].observation.size(); k++) {
      // 检查是否在 frame0 和 frame1 中有观测
      if (sfm_f[j].observation[k].first == frame0) {
        point0 = sfm_f[j].observation[k].second;
        has_0 = true;
      }
      if (sfm_f[j].observation[k].first == frame1) {
        point1 = sfm_f[j].observation[k].second;
        has_1 = true;
      }
    }

    // 如果该特征点在两帧中都可见，则进行三角化，并更新特征点状态
    if (has_0 && has_1) {
      Eigen::Vector3d point_3d;
      triangulatePoint(Pose0, Pose1, point0, point1, point_3d);
      sfm_f[j].state = true;
      sfm_f[j].position[0] = point_3d(0);
      sfm_f[j].position[1] = point_3d(1);
      sfm_f[j].position[2] = point_3d(2);
      std::cout << "trangulated : " << frame1 << "  3d point : " << j << "  "
                << point_3d.transpose() << std::endl;
    }
  }
}

bool GlobalSFM::solveFrameByPnP(Eigen::Matrix3d &R_initial,
                                Eigen::Vector3d &P_initial, int i,
                                std::vector<SFMFeature> &sfm_f) {
  // 收集匹配的2D-3D点对
  std::vector<cv::Point2f> pts_2_vector;
  std::vector<cv::Point3f> pts_3_vector;
  for (int j = 0; j < feature_num; j++) {
    if (sfm_f[j].state != true)
      continue;

    // 该点已完成三角化
    Eigen::Vector2d point2d;
    for (int k = 0; k < (int)sfm_f[j].observation.size(); k++) {
      // 并且该特征点在当前帧i中有观测
      if (sfm_f[j].observation[k].first == i) {
        // 记录其对应的 2D 图像点和 3D 地图点
        Eigen::Vector2d img_pts = sfm_f[j].observation[k].second;
        cv::Point2f pts_2(img_pts(0), img_pts(1));
        pts_2_vector.push_back(pts_2);
        cv::Point3f pts_3(sfm_f[j].position[0], sfm_f[j].position[1],
                          sfm_f[j].position[2]);
        pts_3_vector.push_back(pts_3);
        break;
      }
    }
  }

  // 如果匹配的点对少于 15 个，提示用户移动设备
  if (int(pts_2_vector.size()) < 15) {
    printf("unstable features tracking, please slowly move you device!\n");
    if (int(pts_2_vector.size()) < 10)
      return false;
  }

  cv::Mat r, rvec, t, D, tmp_r;
  cv::eigen2cv(R_initial, tmp_r);
  cv::Rodrigues(tmp_r, rvec);
  cv::eigen2cv(P_initial, t);
  cv::Mat K = (cv::Mat_<double>(3, 3) << 1, 0, 0, 0, 1, 0, 0, 0, 1);
  bool pnp_succ;
  pnp_succ = cv::solvePnP(pts_3_vector, pts_2_vector, K, D, rvec, t, 1);
  if (!pnp_succ) {
    return false;
  }

  // 将结果转换回 Eigen 类型并更新输入参数
  cv::Rodrigues(rvec, r);
  MatrixXd R_pnp;
  cv::cv2eigen(r, R_pnp);
  MatrixXd T_pnp;
  cv::cv2eigen(t, T_pnp);
  R_initial = R_pnp;
  P_initial = T_pnp;
  return true;
}

bool GlobalSFM::construct(int frame_num, Eigen::Quaterniond *q,
                          Eigen::Vector3d *T, int l,
                          const Eigen::Matrix3d relative_R,
                          const Eigen::Vector3d relative_T,
                          Eigen::vector<SFMFeature> &sfm_f,
                          std::map<int, Eigen::Vector3d> &sfm_tracked_points) {
  feature_num = sfm_f.size();

  // 1. 假设已知两帧之间的相对位姿
  // 初始化参考帧 l
  q[l].w() = 1;
  q[l].x() = 0;
  q[l].y() = 0;
  q[l].z() = 0;
  T[l].setZero();

  // 初始化最后一帧，这里围绕imu本身，使用右乘
  q[frame_num - 1] = q[l] * Quaterniond(relative_R);
  T[frame_num - 1] = relative_T;
  std::cout << "init last frame q_l " << q[l].w() << " "
            << q[l].vec().transpose() << std::endl;
  std::cout << "init last frame t_l " << T[l].transpose() << std::endl;

  // 2. rotate to cam frame
  Eigen::Matrix3d c_Rotation[frame_num];
  Eigen::Vector3d c_Translation[frame_num];
  Eigen::Quaterniond c_Quat[frame_num];
  double c_rotation[frame_num][4];
  double c_translation[frame_num][3];
  Eigen::Matrix<double, 3, 4> Pose[frame_num];

  c_Quat[l] = q[l].inverse();
  c_Rotation[l] = c_Quat[l].toRotationMatrix();
  c_Translation[l] = -1 * (c_Rotation[l] * T[l]); // 平移转换到相机坐标系
  Pose[l].block<3, 3>(0, 0) = c_Rotation[l];
  Pose[l].block<3, 1>(0, 3) = c_Translation[l];

  c_Quat[frame_num - 1] = q[frame_num - 1].inverse();
  c_Rotation[frame_num - 1] = c_Quat[frame_num - 1].toRotationMatrix();
  c_Translation[frame_num - 1] =
      -1 *
      (c_Rotation[frame_num - 1] * T[frame_num - 1]); // 平移转换到相机坐标系
  Pose[frame_num - 1].block<3, 3>(0, 0) = c_Rotation[frame_num - 1];
  Pose[frame_num - 1].block<3, 1>(0, 3) = c_Translation[frame_num - 1];

  // 3. 传播式SFM结构，优化当前帧位姿，然后与最后帧进行三角化
  // 1: trangulate between l ----- frame_num - 1
  // 2: solve pnp l + 1; trangulate l + 1 ------- frame_num - 1;
  for (int i = l; i < frame_num - 1; i++) {
    // 从参考帧 l 开始，依次对后续帧 i 进行 PnP 求解当前帧位姿
    if (i > l) {
      Eigen::Matrix3d R_initial = c_Rotation[i - 1];
      Eigen::Vector3d P_initial = c_Translation[i - 1];
      if (!solveFrameByPnP(R_initial, P_initial, i, sfm_f))
        return false;

      // 更新当前帧 i 的位姿
      c_Rotation[i] = R_initial;
      c_Translation[i] = P_initial;
      c_Quat[i] = c_Rotation[i];
      Pose[i].block<3, 3>(0, 0) = c_Rotation[i];
      Pose[i].block<3, 1>(0, 3) = c_Translation[i];
    }

    // 然后使用该帧与最后一帧之间的位姿进行三角化，重建更多三维地图点
    triangulateTwoFrames(i, Pose[i], frame_num - 1, Pose[frame_num - 1], sfm_f);
  }

  // 3: triangulate l-----l+1 l+2 ... frame_num -2
  // 横向传播，利用参考帧 l 和其他帧 i，继续三角化共视点
  for (int i = l + 1; i < frame_num - 1; i++)
    triangulateTwoFrames(l, Pose[l], i, Pose[i], sfm_f);

  // 4: solve pnp l-1; triangulate l-1 ----- l
  //             l-2              l-2 ----- l
  // 反向传播 处理参考帧之前的帧 同样使用 PnP + Triangulation 方法
  for (int i = l - 1; i >= 0; i--) {
    // solve pnp
    Eigen::Matrix3d R_initial = c_Rotation[i + 1];
    Eigen::Vector3d P_initial = c_Translation[i + 1];
    if (!solveFrameByPnP(R_initial, P_initial, i, sfm_f))
      return false;

    c_Rotation[i] = R_initial;
    c_Translation[i] = P_initial;
    c_Quat[i] = c_Rotation[i];
    Pose[i].block<3, 3>(0, 0) = c_Rotation[i];
    Pose[i].block<3, 1>(0, 3) = c_Translation[i];
    // 之前帧与参考帧的三角化
    triangulateTwoFrames(i, Pose[i], l, Pose[l], sfm_f);
  }

  // 5: triangulate all other points
  // 三角化所有剩余可重建的点
  for (int j = 0; j < feature_num; j++) {
    if (sfm_f[j].state == true)
      continue;

    // 该特征点至少被2帧观测
    if ((int)sfm_f[j].observation.size() >= 2) {
      Eigen::Vector2d point0, point1;
      int frame_0 = sfm_f[j].observation[0].first;
      point0 = sfm_f[j].observation[0].second;
      int frame_1 = sfm_f[j].observation.back().first;
      point1 = sfm_f[j].observation.back().second;

      Eigen::Vector3d point_3d;
      // DLT三角化
      triangulatePoint(Pose[frame_0], Pose[frame_1], point0, point1, point_3d);
      sfm_f[j].state = true;
      sfm_f[j].position[0] = point_3d(0);
      sfm_f[j].position[1] = point_3d(1);
      sfm_f[j].position[2] = point_3d(2);
      // cout << "trangulated : " << frame_0 << " " << frame_1 << "  3d point :
      // "  << j << "  " << point_3d.transpose() << endl;
    }
  }

  // 6. full BA 全局BA
  // Ceres Solver 对所有相机位姿和地图点进行非线性优化
  ceres::Problem problem;
  ceres::LocalParameterization *local_parameterization =
      new ceres::QuaternionParameterization();
  // cout << " begin full BA " << endl;
  for (int i = 0; i < frame_num; i++) {
    // double array for ceres
    c_translation[i][0] = c_Translation[i].x();
    c_translation[i][1] = c_Translation[i].y();
    c_translation[i][2] = c_Translation[i].z();
    c_rotation[i][0] = c_Quat[i].w();
    c_rotation[i][1] = c_Quat[i].x();
    c_rotation[i][2] = c_Quat[i].y();
    c_rotation[i][3] = c_Quat[i].z();
    problem.AddParameterBlock(c_rotation[i], 4, local_parameterization);
    problem.AddParameterBlock(c_translation[i], 3);
    if (i == l) {
      problem.SetParameterBlockConstant(c_rotation[i]);
    }
    if (i == l || i == frame_num - 1) {
      problem.SetParameterBlockConstant(c_translation[i]);
    }
  }

  for (int i = 0; i < feature_num; i++) {
    if (sfm_f[i].state != true)
      continue;
    for (int j = 0; j < int(sfm_f[i].observation.size()); j++) {
      int l = sfm_f[i].observation[j].first;
      ceres::CostFunction *cost_function =
          ReprojectionError3D::Create(sfm_f[i].observation[j].second.x(),
                                      sfm_f[i].observation[j].second.y());

      problem.AddResidualBlock(cost_function, NULL, c_rotation[l],
                               c_translation[l], sfm_f[i].position);
    }
  }

  ceres::Solver::Options options;
  options.linear_solver_type = ceres::DENSE_SCHUR;
  // options.minimizer_progress_to_stdout = true;
  options.max_solver_time_in_seconds = 0.2;
  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);
  // std::cout << summary.BriefReport() << "\n";

  if (summary.termination_type == ceres::CONVERGENCE ||
      summary.final_cost < 5e-03) {
    // cout << "vision only BA converge" << endl;
  } else {
    // cout << "vision only BA not converge " << endl;
    return false;
  }

  // update result
  for (int i = 0; i < frame_num; i++) {
    q[i].w() = c_rotation[i][0];
    q[i].x() = c_rotation[i][1];
    q[i].y() = c_rotation[i][2];
    q[i].z() = c_rotation[i][3];
    q[i] = q[i].inverse();
    // cout << "final  q" << " i " << i <<"  " <<q[i].w() << "  " <<
    // q[i].vec().transpose() << endl;
  }

  for (int i = 0; i < frame_num; i++) {
    // 平移转到世界坐标系下
    T[i] = -1 * (q[i] * Vector3d(c_translation[i][0], c_translation[i][1],
                                 c_translation[i][2]));
    // cout << "final  t" << " i " << i <<"  " << T[i](0) <<"  "<< T[i](1) <<"
    // "<< T[i](2) << endl;
  }

  for (int i = 0; i < (int)sfm_f.size(); i++) {
    if (sfm_f[i].state)
      sfm_tracked_points[sfm_f[i].id] = Vector3d(
          sfm_f[i].position[0], sfm_f[i].position[1], sfm_f[i].position[2]);
  }

  return true;
}

} // namespace sensor_lab
