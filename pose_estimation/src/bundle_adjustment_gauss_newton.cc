#include "bundle_adjustment_gauss_newton.h"
#include "transform/calibration_convert.hpp"
#include "transform/cv_eigen_convert.hpp"
#include "transform/eigenGeometryTransfer.hpp"

using namespace SensorLab;

BA_GaussNewton::BA_GaussNewton() {
  feature_detect_ptr_ = std::make_unique<ORB_CV>();
  feature_detect_ptr_->initialization();

  iteration_time_ = 10;
}

BA_GaussNewton::~BA_GaussNewton() {}

void BA_GaussNewton::setIteration(int iteration_time) {
  iteration_time_ = iteration_time;
}

void BA_GaussNewton::inputParams(const char *left_img, const char *right_img,
                                 const char *left_depth_img,
                                 const char *right_depth_img) {
  feature_detect_ptr_->inputParams(left_img, right_img);
  feature_detect_ptr_->run();

  orb_match_result_.clear();
  key_points_img_l_.clear();
  key_points_img_r_.clear();
  feature_detect_ptr_->output(orb_match_result_, key_points_img_l_,
                              key_points_img_r_);

  // 16 unsigned depth image
  left_image_depth_ = cv::imread(left_depth_img, cv::IMREAD_UNCHANGED);
  right_image_depth_ = cv::imread(right_depth_img, cv::IMREAD_UNCHANGED);
}

void BA_GaussNewton::initialization() {}

void BA_GaussNewton::bundleAdjustmentGaussNewton(
    const std::vector<Eigen::Vector3d,
                      Eigen::aligned_allocator<Eigen::Vector3d>> &points_3d,
    const std::vector<Eigen::Vector2d,
                      Eigen::aligned_allocator<Eigen::Vector2d>> &points_2d,
    const cv::Mat &K, Sophus::SE3 &pose) {
  double fx = K.at<double>(0, 0);
  double fy = K.at<double>(1, 1);

  double cost = 0, last_cast = 0;

  for (int iter = 0; iter < iteration_time_; iter++) {
    Eigen::Matrix<double, 6, 6> H =
        Eigen::Matrix<double, 6, 6>::Zero(); // Hessian = J^T * J
    Eigen::Matrix<double, 6, 1> b =
        Eigen::Matrix<double, 6, 1>::Zero(); // bias = -1 * J^T * f(x)
    cost = 0;

    for (int i = 0; i < points_3d.size(); i++) {
      Eigen::Vector3d pc = pose * points_3d[i]; // 3d点左乘李代数se(3)位姿
      auto pixel_p = cam2PixelFrame(cv::Point3d(pc.x(), pc.y(), pc.z()), K);
      Eigen::Vector2d world_point_project_pixel(pixel_p.x, pixel_p.y);

      // 这里是 观测值 - 预测值，jacobian矩阵符号
      // e (2*1)
      Eigen::Vector2d e = points_2d[i] - world_point_project_pixel;

      // update cost
      cost += e.squaredNorm(); // 取两个点之间的距离的平方

      double inv_z = 1.0 / pc[2];
      double inv_z2 = inv_z * inv_z;
      Eigen::Matrix<double, 2, 6> jacobian_mat; // jacobian_mat (2*6)
      // clang-format off
      jacobian_mat << 
      fx * inv_z,
      0,
      (-fx * inv_z2 * pc[0]),
      (-fx * pc[0] * pc[1] * inv_z2),
      fx * (1 - pc[0] * pc[0] * inv_z2),
      (-fx * pc[1] * inv_z2),
      0,
      fy * inv_z,
      (-fy * inv_z2 * pc[0]),
      -1 * fy * (1 + pc[1] * pc[1] * inv_z2),
      fy * pc[0] * pc[1] * inv_z2,
      fy * pc[0] * inv_z2;
      // clang-format on

      jacobian_mat = -1 * jacobian_mat;

      H += jacobian_mat.transpose() * jacobian_mat; // (6*2) * (2*6)
      b += -1 * jacobian_mat.transpose() * e;       // (6*2) * (2*1)
    }

    // [H*Δx = b] solve Δx, Δx(6*1)
    Eigen::Matrix<double, 6, 1> delta_x;
    delta_x = H.ldlt().solve(b);

    if (isnan(delta_x[0])) {
      std::cout << "result is nan!" << std::endl;
      break;
    }

    if (iter > 0 && cost >= last_cast) {
      // cost increase, update is not good
      std::cout << "cost: " << cost << ", last cost: " << last_cast
                << std::endl;
      break;
    }

    // update estimation pose
    // 李代数位姿se(3)通过指数映射为李群SE(3)
    pose = Sophus::SE3::exp(delta_x) * pose;
    last_cast = cost;

    std::cout << "iteration " << iter << " cost=" << std::setprecision(12)
              << cost << ", delta x: " << delta_x.norm()
              << std::endl; // setprecision 指定浮点数字的小数点后要显示的位数
    // Δx 达到收敛标准
    if (delta_x.norm() < 1e-6) {
      // converge 收敛
      break;
    }
  }

  std::cout << "pose by BA gauss-newton: \n" << pose.matrix() << std::endl;
}

void BA_GaussNewton::run() {
  getWorldFramePointsAnd2DPoints();

  std::cout << "using opencv solvePnP to estimate transformation of world "
               "frame to right image"
            << std::endl;
  cv::Mat r, t;
  cv::solvePnP(world_frame_points_, camera_frame_points_,
               camera_intrinsics_mat_, cv::Mat(), r, t);

  cv::Mat rotation_mat;
  cv::Rodrigues(r, rotation_mat); // rotation vector convert to matrix
  std::cout << "rotation_mat:\n"
            << rotation_mat << "\ntranslation:\n"
            << t << std::endl;

  world_3d_points_.clear();
  camera_2d_points_.clear();
  for (size_t i = 0; i < camera_frame_points_.size(); i++) {
    world_3d_points_.push_back(Eigen::Vector3d(world_frame_points_[i].x,
                                               world_frame_points_[i].y,
                                               world_frame_points_[i].z));
    camera_2d_points_.push_back(
        Eigen::Vector2d(camera_frame_points_[i].x, camera_frame_points_[i].y));
  }

  Sophus::SE3 se3_pose;
  bundleAdjustmentGaussNewton(world_3d_points_, camera_2d_points_,
                              camera_intrinsics_mat_, se3_pose);
}

void BA_GaussNewton::getWorldFramePointsAnd2DPoints() {
  world_frame_points_.clear();
  camera_frame_points_.clear();
  for (cv::DMatch &m : orb_match_result_) {
    unsigned short dep = left_image_depth_.ptr<unsigned short>(
        int(key_points_img_l_[m.queryIdx]
                .pt.y))[int(key_points_img_l_[m.queryIdx].pt.x)];

    if (dep == 0)
      continue;

    // TODO:
    float dd = dep / 5000.0;
    cv::Point2d point = pixel2CameraFrame(key_points_img_l_[m.queryIdx].pt,
                                          camera_intrinsics_mat_);

    // world frame point
    world_frame_points_.push_back(cv::Point3f(point.x * dd, point.y * dd, dd));

    // 2d points
    camera_frame_points_.push_back(
        cv::Point2f(key_points_img_r_[m.trainIdx].pt));
  }

  std::cout << "3d points size: " << world_frame_points_.size()
            << ", 2d points size: " << camera_frame_points_.size() << std::endl;

  std::cout << "get 3d world frame points and 2d points" << std::endl;
}

void BA_GaussNewton::inputCameraIntrinsics(
    const std::vector<double> &intrinsics) {
  assert(intrinsics.size() == 4);
  camera_intrinsics_mat_ =
      (cv::Mat_<double>(3, 3) << intrinsics[0], 0, intrinsics[2], 0,
       intrinsics[1], intrinsics[3], 0, 0, 1);
}
