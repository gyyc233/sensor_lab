#include "based_imu_pre_integration_gins.h"
#include <glog/logging.h>

#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/core/robust_kernel.h>
#include <g2o/solvers/dense/linear_solver_dense.h>

#include "g2o_preintegration_types.h"
#include "g2o_types/g2o_types.h"

namespace sad {
GinsImuPreIntegration::GinsImuPreIntegration(Options options)
    : options_(options) {
  SetOptions(options_);
}

GinsImuPreIntegration::~GinsImuPreIntegration() {}

void GinsImuPreIntegration::SetOptions(
    sad::GinsImuPreIntegration::Options options) {
  // add gryo var
  double bg_rw2 = 1.0 / (options_.bias_gyro_var * options_.bias_gyro_var);
  options_.bg_rw_info.diagonal() << bg_rw2, bg_rw2, bg_rw2;

  // add acce var
  double ba_rw2 = 1.0 / (options_.bias_acce_var * options_.bias_acce_var);
  options_.ba_rw_info.diagonal() << ba_rw2, ba_rw2, ba_rw2;

  // add gnss var
  double gp2 = options_.gnss_pos_noise * options_.gnss_pos_noise;
  double gh2 = options_.gnss_height_noise * options_.gnss_height_noise;
  double ga2 = options_.gnss_ang_noise * options_.gnss_ang_noise;

  // 初始化gnss信息矩阵
  options_.gnss_info.diagonal() << 1.0 / ga2, 1.0 / ga2, 1.0 / ga2, 1.0 / gp2,
      1.0 / gp2, 1.0 / gh2;

  pre_integ_ = std::make_shared<IMUPreintegration>(options_.preinteg_options);

  // add odom var
  double o2 = 1.0 / (options_.odom_var * options_.odom_var);
  // 初始化odom信息矩阵
  options_.odom_info.diagonal() << o2, o2, o2;

  prior_info_.block<6, 6>(9, 9) = Mat6d ::Identity() * 1e6;

  if (this_frame_) {
    this_frame_->bg_ = options_.preinteg_options.init_bg_;
    this_frame_->ba_ = options_.preinteg_options.init_ba_;
  }
}

void GinsImuPreIntegration::AddImu(const IMU &imu) {
  if (first_gnss_received_ && first_imu_received_) {
    pre_integ_->Integrate(imu, imu.timestamp_ - last_imu_.timestamp_);
  }

  first_imu_received_ = true;
  last_imu_ = imu;
  current_time_ = imu.timestamp_;
}

void GinsImuPreIntegration::AddGnss(const GNSS &gnss) {
  this_frame_ = std::make_shared<NavStated>(current_time_);
  this_gnss_ = gnss;

  // 初始化处理
  if (!first_gnss_received_) {
    if (!gnss.heading_valid_) {
      // 要求首个GNSS必须有航向
      return;
    }

    // 首个gnss信号，将初始pose设置为该gnss信号
    // 当首个带姿态的 GNSS
    // 数据到来时，使用该数据初始化初始位姿，初始速度为0，初始的零偏等于预积分类中的零偏（即
    // IMU 静止初始化得到的零偏）
    this_frame_->timestamp_ = gnss.unix_time_;
    this_frame_->p_ = gnss.utm_pose_.translation();
    this_frame_->R_ = gnss.utm_pose_.so3();
    this_frame_->v_.setZero();
    this_frame_->bg_ = options_.preinteg_options.init_bg_;
    this_frame_->ba_ = options_.preinteg_options.init_ba_;

    pre_integ_ = std::make_shared<IMUPreintegration>(options_.preinteg_options);

    last_frame_ = this_frame_;
    last_gnss_ = this_gnss_;
    first_gnss_received_ = true;
    current_time_ = gnss.unix_time_;
    return;
  }

  // 积分到GNSS时刻
  // 在 GNSS 数据到达时，先使用 last_imu_ 数据预积分到 GNSS 时刻
  pre_integ_->Integrate(last_imu_, gnss.unix_time_ - current_time_);

  current_time_ = gnss.unix_time_;
  *this_frame_ = pre_integ_->Predict(*last_frame_, options_.gravity);

  // 执行优化
  Optimize();

  last_frame_ = this_frame_;
  last_gnss_ = this_gnss_;
}

void GinsImuPreIntegration::AddOdom(const sad::Odom &odom) {
  last_odom_ = odom;
  last_odom_set_ = true;
}

NavStated GinsImuPreIntegration::GetState() const {
  if (this_frame_ == nullptr) {
    return {};
  }

  if (pre_integ_ == nullptr) {
    return *this_frame_;
  }

  return pre_integ_->Predict(*this_frame_, options_.gravity);
}

void GinsImuPreIntegration::Optimize() {
  if (pre_integ_->dt_ < 1e-3) {
    // 未得到积分
    return;
  }

  // using BlockSolverType = g2o::BlockSolverX;
  // using LinearSolverType =
  // g2o::LinearSolverEigen<BlockSolverType::PoseMatrixType>;s

  // auto* solver = new g2o::OptimizationAlgorithmLevenberg(
  //     g2o::make_unique<BlockSolverType>(g2o::make_unique<LinearSolverType>()));

  // 创建可变尺寸的 BlockSolver
  typedef g2o::BlockSolverX::LinearSolverType BlockSolverType;
  // 线性求解器类型
  typedef g2o::LinearSolverDense<g2o::BlockSolverX::PoseMatrixType>
      LinearSolverType;
  // 求解方式：LinearSolverDense dense cholesky分解法
  LinearSolverType *linear_solver =
      new g2o::LinearSolverDense<g2o::BlockSolverX::PoseMatrixType>();

  // 创建块求解器
  g2o::BlockSolverX *solver_ptr = new g2o::BlockSolverX(linear_solver);

  // 优化算法
  g2o::OptimizationAlgorithmLevenberg *solver =
      new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
  // 创建稀疏求解器
  g2o::SparseOptimizer optimizer;
  optimizer.setAlgorithm(solver);
  // 需要定义各种顶点和边

  // 上时刻顶点， pose, v, bg, ba
  auto v0_pose = new VertexPose();
  v0_pose->setId(0);
  v0_pose->setEstimate(last_frame_->GetSE3());
  optimizer.addVertex(v0_pose);

  auto v0_vel = new VertexVelocity();
  v0_vel->setId(1);
  v0_vel->setEstimate(last_frame_->v_);
  optimizer.addVertex(v0_vel);

  auto v0_bg = new VertexGyroBias();
  v0_bg->setId(2);
  v0_bg->setEstimate(last_frame_->bg_);
  optimizer.addVertex(v0_bg);

  auto v0_ba = new VertexAccBias();
  v0_ba->setId(3);
  v0_ba->setEstimate(last_frame_->ba_);
  optimizer.addVertex(v0_ba);

  // 本时刻顶点，pose, v, bg, ba
  auto v1_pose = new VertexPose();
  v1_pose->setId(4);
  v1_pose->setEstimate(this_frame_->GetSE3());
  optimizer.addVertex(v1_pose);

  auto v1_vel = new VertexVelocity();
  v1_vel->setId(5);
  v1_vel->setEstimate(this_frame_->v_);
  optimizer.addVertex(v1_vel);

  auto v1_bg = new VertexGyroBias();
  v1_bg->setId(6);
  v1_bg->setEstimate(this_frame_->bg_);
  optimizer.addVertex(v1_bg);

  auto v1_ba = new VertexAccBias();
  v1_ba->setId(7);
  v1_ba->setEstimate(this_frame_->ba_);
  optimizer.addVertex(v1_ba);

  // 预积分边
  auto edge_inertial = new EdgeInertial(pre_integ_, options_.gravity);
  edge_inertial->setVertex(0, v0_pose);
  edge_inertial->setVertex(1, v0_vel);
  edge_inertial->setVertex(2, v0_bg);
  edge_inertial->setVertex(3, v0_ba);
  edge_inertial->setVertex(4, v1_pose);
  edge_inertial->setVertex(5, v1_vel);

  // Huber核函数: 数据中异常值较温和但数量较多
  // Cauchy核函数: 异常值的数量相对较少但幅度较大(即噪声点极端但不频繁)
  auto *rk = new g2o::RobustKernelHuber();
  rk->setDelta(200.0);
  edge_inertial->setRobustKernel(rk);
  optimizer.addEdge(edge_inertial);

  // 两个零偏随机游走
  auto *edge_gyro_rw = new EdgeGyroRW();
  edge_gyro_rw->setVertex(0, v0_bg);
  edge_gyro_rw->setVertex(1, v1_bg);
  edge_gyro_rw->setInformation(options_.bg_rw_info);
  optimizer.addEdge(edge_gyro_rw);

  auto *edge_acc_rw = new EdgeAccRW();
  edge_acc_rw->setVertex(0, v0_ba);
  edge_acc_rw->setVertex(1, v1_ba);
  edge_acc_rw->setInformation(options_.ba_rw_info);
  optimizer.addEdge(edge_acc_rw);

  // 上时刻先验
  auto *edge_prior = new EdgePriorPoseNavState(*last_frame_, prior_info_);
  edge_prior->setVertex(0, v0_pose);
  edge_prior->setVertex(1, v0_vel);
  edge_prior->setVertex(2, v0_bg);
  edge_prior->setVertex(3, v0_ba);
  optimizer.addEdge(edge_prior);

  // GNSS边
  auto edge_gnss0 = new EdgeGNSS(v0_pose, last_gnss_.utm_pose_);
  edge_gnss0->setInformation(options_.gnss_info);
  optimizer.addEdge(edge_gnss0);

  auto edge_gnss1 = new EdgeGNSS(v1_pose, this_gnss_.utm_pose_);
  edge_gnss1->setInformation(options_.gnss_info);
  optimizer.addEdge(edge_gnss1);

  // Odom边
  EdgeEncoder3D *edge_odom = nullptr;
  Vec3d vel_world = Vec3d::Zero();
  Vec3d vel_odom = Vec3d::Zero();
  if (last_odom_set_) {
    // velocity obs
    double velo_l = options_.wheel_radius * last_odom_.left_pulse_ /
                    options_.circle_pulse * 2 * M_PI / options_.odom_span;
    double velo_r = options_.wheel_radius * last_odom_.right_pulse_ /
                    options_.circle_pulse * 2 * M_PI / options_.odom_span;
    double average_vel = 0.5 * (velo_l + velo_r);
    vel_odom = Vec3d(average_vel, 0.0, 0.0);
    vel_world = this_frame_->R_ * vel_odom;

    edge_odom = new EdgeEncoder3D(v1_vel, vel_world);
    edge_odom->setInformation(options_.odom_info);
    optimizer.addEdge(edge_odom);

    // 重置odom数据到达标志位，等待最新的odom数据
    last_odom_set_ = false;
  }

  optimizer.setVerbose(options_.verbose);
  optimizer.initializeOptimization();
  optimizer.optimize(20);

  if (options_.verbose) {
    // 获取结果，统计各类误差
    LOG(INFO) << "chi2/error: ";
    LOG(INFO) << "preintegration: " << edge_inertial->chi2() << "/"
              << edge_inertial->error().transpose();
    // LOG(INFO) << "gnss0: " << edge_gnss0->chi2() << ", " <<
    edge_gnss0->error().transpose();
    LOG(INFO) << "gnss1: " << edge_gnss1->chi2() << ", "
              << edge_gnss1->error().transpose();
    LOG(INFO) << "bias: " << edge_gyro_rw->chi2() << "/"
              << edge_acc_rw->error().transpose();
    LOG(INFO) << "prior: " << edge_prior->chi2() << "/"
              << edge_prior->error().transpose();
    if (edge_odom) {
      LOG(INFO) << "body vel: "
                << (v1_pose->estimate().so3().inverse() * v1_vel->estimate())
                       .transpose();
      LOG(INFO) << "meas: " << vel_odom.transpose();
      LOG(INFO) << "odom: " << edge_odom->chi2() << "/"
                << edge_odom->error().transpose();
    }
  }

  last_frame_->R_ = v0_pose->estimate().so3();
  last_frame_->p_ = v0_pose->estimate().translation();
  last_frame_->v_ = v0_vel->estimate();
  last_frame_->bg_ = v0_bg->estimate();
  last_frame_->ba_ = v0_ba->estimate();

  this_frame_->R_ = v1_pose->estimate().so3();
  this_frame_->p_ = v1_pose->estimate().translation();
  this_frame_->v_ = v1_vel->estimate();
  this_frame_->bg_ = v1_bg->estimate();
  this_frame_->ba_ = v1_ba->estimate();

  // 重置integ
  options_.preinteg_options.init_bg_ = this_frame_->bg_;
  options_.preinteg_options.init_ba_ = this_frame_->ba_;
  pre_integ_ = std::make_shared<IMUPreintegration>(options_.preinteg_options);
}

} // namespace sad
