#include "initial_imu_camera_ex_rotation.h"
#include <iostream>

namespace sensor_lab {
InitialEXRotation::InitialEXRotation() {
  frame_count = 0;
  Rc.push_back(Eigen::Matrix3d::Identity());
  Rc_g.push_back(Eigen::Matrix3d::Identity());
  Rimu.push_back(Eigen::Matrix3d::Identity());
  ric = Eigen::Matrix3d::Identity();
}

bool InitialEXRotation::CalibrationExRotation(
    std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> corres,
    Eigen::Quaterniond delta_q_imu, Eigen::Matrix3d &calib_ric_result) {
  frame_count++;                        // 当前处理帧数
  Rc.push_back(solveRelativeR(corres)); // 计算视觉估计的真贱相对旋转
  Rimu.push_back(delta_q_imu.toRotationMatrix()); // imu测量的帧间旋转

  // 在相机坐标系下的帧间旋转预测值 = ric.inverse() * delta_q_imu * ric
  // 右侧使用了四元数旋转的乘法
  Rc_g.push_back(ric.inverse() * delta_q_imu * ric);

  // 如果Rc与Rc_g几乎一致，说明我们的外参估计准确

  Eigen::MatrixXd A(frame_count * 4, 4); // 每4行对应一帧的旋转
  A.setZero();
  int sum_ok = 0;
  for (int i = 1; i <= frame_count; i++) {
    Eigen::Quaterniond r1(Rc[i]);
    Eigen::Quaterniond r2(Rc_g[i]);

    // Quaternion::angularDistance() 方法，它返回两个四元数所代表旋转之间的夹角
    double angular_distance = 180 / M_PI * r1.angularDistance(r2);
    std::cout << "angular_distance: " << angular_distance << std::endl;

    // 这个角度很小（比如 < 5°），说明两者匹配良好，当前的外参估计可能是准确的
    double huber = angular_distance > 5.0 ? 5.0 / angular_distance : 1.0;
    ++sum_ok;
    Eigen::Matrix4d L, R;

    // 构造四元数左乘矩阵 L （视觉估计）
    double w = Eigen::Quaterniond(Rc[i]).w();
    Eigen::Vector3d q = Eigen::Quaterniond(Rc[i]).vec();
    L.block<3, 3>(0, 0) =
        w * Eigen::Matrix3d::Identity() + Utility::skewSymmetric(q);
    L.block<3, 1>(0, 3) = q;
    L.block<1, 3>(3, 0) = -q.transpose();
    L(3, 3) = w;

    // 构造四元数右乘矩阵 R （IMU估计）
    Eigen::Quaterniond R_ij(Rimu[i]);
    w = R_ij.w();
    q = R_ij.vec();
    R.block<3, 3>(0, 0) =
        w * Eigen::Matrix3d::Identity() - Utility::skewSymmetric(q);
    R.block<3, 1>(0, 3) = q;
    R.block<1, 3>(3, 0) = -q.transpose();
    R(3, 3) = w;

    // 构造误差项L-R 视觉估计与IMU估计的差异
    A.block<4, 4>((i - 1) * 4, 0) = huber * (L - R);
  }

  // 对A做SVD分解，最小奇异值对应的右奇异向量即为最优四元数
  Eigen::JacobiSVD<Eigen::MatrixXd> svd(A, Eigen::ComputeFullU |
                                               Eigen::ComputeFullV);
  Eigen::Matrix<double, 4, 1> x = svd.matrixV().col(3);
  Eigen::Quaterniond estimated_R(x);
  // 转换为旋转矩阵后取逆得到最终的 imu-camera 旋转矩阵
  ric = estimated_R.toRotationMatrix().inverse();
  // cout << svd.singularValues().transpose() << endl;
  // cout << ric << endl;

  // 判断是否满足标定条件
  Eigen::Vector3d ric_cov;
  ric_cov = svd.singularValues().tail<3>(); // 取后三个奇异值作为估计不确定性
  // 如果帧数足够（超过滑动窗口大小）且估计置信度高（第二个奇异值大于阈值），则返回成功
  if (frame_count >= window_size && ric_cov(1) > 0.25) {
    calib_ric_result = ric;
    return true;
  } else
    return false;
}

Eigen::Matrix3d InitialEXRotation::solveRelativeR(
    const std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> &corres) {
  if (corres.size() >= 9) {
    std::vector<cv::Point2f> ll, rr;
    for (int i = 0; i < int(corres.size()); i++) {
      ll.push_back(cv::Point2f(corres[i].first(0), corres[i].first(1)));
      rr.push_back(cv::Point2f(corres[i].second(0), corres[i].second(1)));
    }

    // 估计基础矩阵 E（这里其实更准确地说是 Fundamental
    // Matrix，但在内参已知时等价于 Essential Matrix）
    cv::Mat E = cv::findFundamentalMat(ll, rr);
    cv::Mat_<double> R1, R2, t1, t2;
    decomposeE(E, R1, R2, t1, t2);

    if (determinant(R1) + 1.0 < 1e-09) {
      E = -E;
      decomposeE(E, R1, R2, t1, t2);
    }

    // 对每种组合进行三角化测试，选择有最多点在相机前的R
    double ratio1 = max(testTriangulation(ll, rr, R1, t1),
                        testTriangulation(ll, rr, R1, t2));
    double ratio2 = max(testTriangulation(ll, rr, R2, t1),
                        testTriangulation(ll, rr, R2, t2));
    cv::Mat_<double> ans_R_cv = ratio1 > ratio2 ? R1 : R2;

    Eigen::Matrix3d ans_R_eigen;
    for (int i = 0; i < 3; i++)
      for (int j = 0; j < 3; j++)
        ans_R_eigen(j, i) = ans_R_cv(i, j);
    return ans_R_eigen;
  }
  return Eigen::Matrix3d::Identity();
}

double InitialEXRotation::testTriangulation(const std::vector<cv::Point2f> &l,
                                            const std::vector<cv::Point2f> &r,
                                            cv::Mat_<double> R,
                                            cv::Mat_<double> t) {
  cv::Mat pointcloud;
  cv::Matx34f P = cv::Matx34f(1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0);
  cv::Matx34f P1 =
      cv::Matx34f(R(0, 0), R(0, 1), R(0, 2), t(0), R(1, 0), R(1, 1), R(1, 2),
                  t(1), R(2, 0), R(2, 1), R(2, 2), t(2));
  cv::triangulatePoints(P, P1, l, r, pointcloud);
  int front_count = 0;
  // 遍历所有 3D 点，判断是否在相机前方
  for (int i = 0; i < pointcloud.cols; i++) {
    double normal_factor = pointcloud.col(i).at<float>(3);

    cv::Mat_<double> p_3d_l = cv::Mat(P) * (pointcloud.col(i) / normal_factor);
    cv::Mat_<double> p_3d_r = cv::Mat(P1) * (pointcloud.col(i) / normal_factor);
    if (p_3d_l(2) > 0 && p_3d_r(2) > 0)
      front_count++;
  }

  std::cout << "MotionEstimator: " << 1.0 * front_count / pointcloud.cols
            << std::endl;
  // 在两个相机前都可见的点占总点数的比例
  return 1.0 * front_count / pointcloud.cols;
}

void InitialEXRotation::decomposeE(cv::Mat E, cv::Mat_<double> &R1,
                                   cv::Mat_<double> &R2, cv::Mat_<double> &t1,
                                   cv::Mat_<double> &t2) {
  cv::SVD svd(E, cv::SVD::MODIFY_A);
  cv::Matx33d W(0, -1, 0, 1, 0, 0, 0, 0, 1);
  cv::Matx33d Wt(0, 1, 0, -1, 0, 0, 0, 0, 1);
  R1 = svd.u * cv::Mat(W) * svd.vt;
  R2 = svd.u * cv::Mat(Wt) * svd.vt;
  t1 = svd.u.col(2);
  t2 = -svd.u.col(2);
}

} // namespace sensor_lab
