#include "projection_factor.h"
#include <iostream>

namespace sensor_lab {

Eigen::Matrix2d ProjectionFactor::sqrt_info;
double ProjectionFactor::sum_t;

ProjectionFactor::ProjectionFactor(const Eigen::Vector3d &_pts_i,
                                   const Eigen::Vector3d &_pts_j)
    : pts_i(_pts_i), pts_j(_pts_j) {
// 若启用球面误差模型则需要构建切平面基底
// 适用于深度未知或非常大的情况，能提升数值稳定性
#ifdef UNIT_SPHERE_ERROR
  Eigen::Vector3d b1, b2;
  Eigen::Vector3d a = pts_j.normalized();
  Eigen::Vector3d tmp(0, 0, 1);
  if (a == tmp)
    tmp << 1, 0, 0;
  b1 = (tmp - a * (a.transpose() * tmp)).normalized();
  b2 = a.cross(b1);
  tangent_base.block<1, 3>(0, 0) = b1.transpose();
  tangent_base.block<1, 3>(1, 0) = b2.transpose();
#endif
};

bool ProjectionFactor::Evaluate(double const *const *parameters,
                                double *residuals, double **jacobians) const {
  TicToc tic_toc;
  // i frame pose
  Eigen::Vector3d Pi(parameters[0][0], parameters[0][1], parameters[0][2]);
  Eigen::Quaterniond Qi(parameters[0][6], parameters[0][3], parameters[0][4],
                        parameters[0][5]);

  // j frame pose
  Eigen::Vector3d Pj(parameters[1][0], parameters[1][1], parameters[1][2]);
  Eigen::Quaterniond Qj(parameters[1][6], parameters[1][3], parameters[1][4],
                        parameters[1][5]);

  // imu-camera extrinsic params [tx, ty, tz, qx, qy, qz, qw]
  Eigen::Vector3d tic(parameters[2][0], parameters[2][1], parameters[2][2]);
  Eigen::Quaterniond qic(parameters[2][6], parameters[2][3], parameters[2][4],
                         parameters[2][5]);

  // 特征点在i帧上的逆深度
  double inv_dep_i = parameters[3][0];

  Eigen::Vector3d pts_camera_i = pts_i / inv_dep_i; // 归一化相机坐标转相机坐标
  Eigen::Vector3d pts_imu_i = qic * pts_camera_i + tic; // 转到i帧imu坐标系下
  Eigen::Vector3d pts_w = Qi * pts_imu_i + Pi; // i帧imu坐标系转到世界坐标系

  Eigen::Vector3d pts_imu_j =
      Qj.inverse() * (pts_w - Pj); // 世界坐标系转到j帧坐标系下
  Eigen::Vector3d pts_camera_j =
      qic.inverse() * (pts_imu_j - tic); // 转到j帧时的相机坐标系下
  Eigen::Map<Eigen::Vector2d> residual(residuals);

  // 归一化后计算残差（像素座标对相机坐标求偏导）
#ifdef UNIT_SPHERE_ERROR
  residual = tangent_base * (pts_camera_j.normalized() - pts_j.normalized());
#else
  // pts_i 和 pts_j
  // 已经是归一化相机坐标，相机内参已在前端去畸变和坐标转换时应用过
  double dep_j = pts_camera_j.z();
  residual = (pts_camera_j / dep_j).head<2>() - pts_j.head<2>();
#endif

  // 加权残差
  residual = sqrt_info * residual;

  if (jacobians) {
    Eigen::Matrix3d Ri = Qi.toRotationMatrix();
    Eigen::Matrix3d Rj = Qj.toRotationMatrix();
    Eigen::Matrix3d ric = qic.toRotationMatrix();
    Eigen::Matrix<double, 2, 3> reduce(2, 3);
#ifdef UNIT_SPHERE_ERROR
    double norm = pts_camera_j.norm();
    Eigen::Matrix3d norm_jaco;
    double x1, x2, x3;
    x1 = pts_camera_j(0);
    x2 = pts_camera_j(1);
    x3 = pts_camera_j(2);
    norm_jaco << 1.0 / norm - x1 * x1 / pow(norm, 3), -x1 * x2 / pow(norm, 3),
        -x1 * x3 / pow(norm, 3), -x1 * x2 / pow(norm, 3),
        1.0 / norm - x2 * x2 / pow(norm, 3), -x2 * x3 / pow(norm, 3),
        -x1 * x3 / pow(norm, 3), -x2 * x3 / pow(norm, 3),
        1.0 / norm - x3 * x3 / pow(norm, 3);
    reduce = tangent_base * norm_jaco;
#else
    // [1/z,0,-u/z^2]
    // [0,1/z,-v/z^2]
    reduce << 1. / dep_j, 0, -pts_camera_j(0) / (dep_j * dep_j), 0, 1. / dep_j,
        -pts_camera_j(1) / (dep_j * dep_j);
#endif
    reduce = sqrt_info * reduce;

    // 重投影误差对帧 i 位姿（Pose_i）[tx, ty, tz, qx, qy, qz, qw]的雅可比矩阵
    // 即残差对 Pose_i 的偏导数
    if (jacobians[0]) {
      // 如果用户请求计算雅可比矩阵（即 jacobians != nullptr 且 jacobians[0] !=
      // nullptr），则进入计算流程
      Eigen::Map<Eigen::Matrix<double, 2, 7, Eigen::RowMajor>> jacobian_pose_i(
          jacobians[0]);

      // 由于四元数是4维，所以后6维用于构建局部更新空间（由
      // LocalParameterization 处理），最后一维为常量或忽略
      Eigen::Matrix<double, 3, 6> jaco_i;
      jaco_i.leftCols<3>() = ric.transpose() * Rj.transpose();
      jaco_i.rightCols<3>() = ric.transpose() * Rj.transpose() * Ri *
                              -Utility::skewSymmetric(pts_imu_i);

      jacobian_pose_i.leftCols<6>() = reduce * jaco_i;
      jacobian_pose_i.rightCols<1>().setZero();
    }

    // 即残差对 Pose_j 的偏导数
    if (jacobians[1]) {
      Eigen::Map<Eigen::Matrix<double, 2, 7, Eigen::RowMajor>> jacobian_pose_j(
          jacobians[1]);

      Eigen::Matrix<double, 3, 6> jaco_j;
      jaco_j.leftCols<3>() = ric.transpose() * -Rj.transpose();
      jaco_j.rightCols<3>() =
          ric.transpose() * Utility::skewSymmetric(pts_imu_j);

      jacobian_pose_j.leftCols<6>() = reduce * jaco_j;
      jacobian_pose_j.rightCols<1>().setZero();
    }

    // 即残差（）对 imu-camera extrinsic 的偏导数
    if (jacobians[2]) {
      Eigen::Map<Eigen::Matrix<double, 2, 7, Eigen::RowMajor>> jacobian_ex_pose(
          jacobians[2]);

      // 定义一个 3x6 的中间矩阵，表示从 Extrinsic 到 3D 点的导数
      Eigen::Matrix<double, 3, 6> jaco_ex;
      // 前三列 平移部分导数
      jaco_ex.leftCols<3>() =
          ric.transpose() * (Rj.transpose() * Ri - Eigen::Matrix3d::Identity());
      Eigen::Matrix3d tmp_r = ric.transpose() * Rj.transpose() * Ri * ric;
      // 后三列 旋转部分导数
      jaco_ex.rightCols<3>() =
          -tmp_r * Utility::skewSymmetric(pts_camera_i) +
          Utility::skewSymmetric(tmp_r * pts_camera_i) +
          Utility::skewSymmetric(ric.transpose() *
                                 (Rj.transpose() * (Ri * tic + Pi - Pj) - tic));

      jacobian_ex_pose.leftCols<6>() = reduce * jaco_ex;
      jacobian_ex_pose.rightCols<1>().setZero();
    }

    // 残差对逆深度的偏导数
    if (jacobians[3]) {
      Eigen::Map<Eigen::Vector2d> jacobian_feature(jacobians[3]);
#if 1
      jacobian_feature = reduce * ric.transpose() * Rj.transpose() * Ri * ric *
                         pts_i * -1.0 / (inv_dep_i * inv_dep_i);
#else
      jacobian_feature =
          reduce * ric.transpose() * Rj.transpose() * Ri * ric * pts_i;
#endif
    }
  }
  sum_t += tic_toc.toc();

  return true;
}

void ProjectionFactor::check(double **parameters) {
  double *res = new double[15];
  double **jaco = new double *[4];
  jaco[0] = new double[2 * 7];
  jaco[1] = new double[2 * 7];
  jaco[2] = new double[2 * 7];
  jaco[3] = new double[2 * 1];
  Evaluate(parameters, res, jaco);
  puts("check begins");

  puts("my");

  std::cout << Eigen::Map<Eigen::Matrix<double, 2, 1>>(res).transpose()
            << std::endl
            << std::endl;
  std::cout << Eigen::Map<Eigen::Matrix<double, 2, 7, Eigen::RowMajor>>(jaco[0])
            << std::endl
            << std::endl;
  std::cout << Eigen::Map<Eigen::Matrix<double, 2, 7, Eigen::RowMajor>>(jaco[1])
            << std::endl
            << std::endl;
  std::cout << Eigen::Map<Eigen::Matrix<double, 2, 7, Eigen::RowMajor>>(jaco[2])
            << std::endl
            << std::endl;
  std::cout << Eigen::Map<Eigen::Vector2d>(jaco[3]) << std::endl << std::endl;

  Eigen::Vector3d Pi(parameters[0][0], parameters[0][1], parameters[0][2]);
  Eigen::Quaterniond Qi(parameters[0][6], parameters[0][3], parameters[0][4],
                        parameters[0][5]);

  Eigen::Vector3d Pj(parameters[1][0], parameters[1][1], parameters[1][2]);
  Eigen::Quaterniond Qj(parameters[1][6], parameters[1][3], parameters[1][4],
                        parameters[1][5]);

  Eigen::Vector3d tic(parameters[2][0], parameters[2][1], parameters[2][2]);
  Eigen::Quaterniond qic(parameters[2][6], parameters[2][3], parameters[2][4],
                         parameters[2][5]);
  double inv_dep_i = parameters[3][0];

  Eigen::Vector3d pts_camera_i = pts_i / inv_dep_i;
  Eigen::Vector3d pts_imu_i = qic * pts_camera_i + tic;
  Eigen::Vector3d pts_w = Qi * pts_imu_i + Pi;
  Eigen::Vector3d pts_imu_j = Qj.inverse() * (pts_w - Pj);
  Eigen::Vector3d pts_camera_j = qic.inverse() * (pts_imu_j - tic);

  Eigen::Vector2d residual;
#ifdef UNIT_SPHERE_ERROR
  residual = tangent_base * (pts_camera_j.normalized() - pts_j.normalized());
#else
  double dep_j = pts_camera_j.z();
  residual = (pts_camera_j / dep_j).head<2>() - pts_j.head<2>();
#endif
  residual = sqrt_info * residual;

  puts("num");
  std::cout << residual.transpose() << std::endl;

  // 构建数值差分雅可比
  const double eps = 1e-6;
  Eigen::Matrix<double, 2, 19> num_jacobian;
  for (int k = 0; k < 19; k++) {
    Eigen::Vector3d Pi(parameters[0][0], parameters[0][1], parameters[0][2]);
    Eigen::Quaterniond Qi(parameters[0][6], parameters[0][3], parameters[0][4],
                          parameters[0][5]);

    Eigen::Vector3d Pj(parameters[1][0], parameters[1][1], parameters[1][2]);
    Eigen::Quaterniond Qj(parameters[1][6], parameters[1][3], parameters[1][4],
                          parameters[1][5]);

    Eigen::Vector3d tic(parameters[2][0], parameters[2][1], parameters[2][2]);
    Eigen::Quaterniond qic(parameters[2][6], parameters[2][3], parameters[2][4],
                           parameters[2][5]);
    double inv_dep_i = parameters[3][0];

    int a = k / 3, b = k % 3;
    Eigen::Vector3d delta = Eigen::Vector3d(b == 0, b == 1, b == 2) * eps;

    // 对每个参数做微小扰动
    if (a == 0)
      Pi += delta;
    else if (a == 1)
      Qi = Qi * Utility::deltaQ(delta);
    else if (a == 2)
      Pj += delta;
    else if (a == 3)
      Qj = Qj * Utility::deltaQ(delta);
    else if (a == 4)
      tic += delta;
    else if (a == 5)
      qic = qic * Utility::deltaQ(delta);
    else if (a == 6)
      inv_dep_i += delta.x();

    // 重新计算残差
    Eigen::Vector3d pts_camera_i = pts_i / inv_dep_i;
    Eigen::Vector3d pts_imu_i = qic * pts_camera_i + tic;
    Eigen::Vector3d pts_w = Qi * pts_imu_i + Pi;
    Eigen::Vector3d pts_imu_j = Qj.inverse() * (pts_w - Pj);
    Eigen::Vector3d pts_camera_j = qic.inverse() * (pts_imu_j - tic);

    Eigen::Vector2d tmp_residual;
#ifdef UNIT_SPHERE_ERROR
    tmp_residual =
        tangent_base * (pts_camera_j.normalized() - pts_j.normalized());
#else
    double dep_j = pts_camera_j.z();
    tmp_residual = (pts_camera_j / dep_j).head<2>() - pts_j.head<2>();
#endif
    tmp_residual = sqrt_info * tmp_residual;
    num_jacobian.col(k) = (tmp_residual - residual) / eps;
  }
  std::cout << num_jacobian << std::endl;
}

} // namespace sensor_lab
