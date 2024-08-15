#include "bundle_adjustment_g2o.h"

using namespace SensorLab;

void VertexPose::setToOriginImpl() {
  // // 重置，设定被优化变量的原始值
  _estimate = Sophus::SE3();
}

void VertexPose::oplusImpl(const double *update) {
  Eigen::Matrix<double, 6, 1> update_eigen;
  update_eigen << update[0], update[1], update[2], update[3], update[4],
      update[5];
  // Sophus::SE3::exp
  // 李代数-->指数映射为旋转向量-->转为单位四元数-->李群(RT矩阵)
  _estimate = Sophus::SE3::exp(update_eigen) * _estimate;
}

bool VertexPose::write(std::ostream &out) const { return false; }

bool VertexPose::read(std::istream &in) { return false; }

EdgeProjection::EdgeProjection(const Eigen::Vector3d &pos,
                               const Eigen::Matrix3d &K)
    : _pos3d(pos), _K(K) {}

void EdgeProjection::computeError() {
  const VertexPose *v = static_cast<VertexPose *>(_vertices[0]);
  Sophus::SE3 T = v->estimate();
  Eigen::Vector3d pos_pixel = _K * (T * _pos3d);
  pos_pixel /= pos_pixel[2];
  _error = _measurement - pos_pixel.head<2>();
}

void EdgeProjection::linearizeOplus() {
  const VertexPose *v = static_cast<VertexPose *>(_vertices[0]);
  Sophus::SE3 T = v->estimate();
  Eigen::Vector3d pos_cam = T * _pos3d;
  double fx = _K(0, 0);
  double fy = _K(1, 1);
  double X = pos_cam[0];
  double Y = pos_cam[1];
  double Z = pos_cam[2];
  double Z2 = Z * Z;
  _jacobianOplusXi << -fx / Z, 0, fx * X / Z2, fx * X * Y / Z2,
      -fx - fx * X * X / Z2, fx * Y / Z, 0, -fy / Z, fy * Y / (Z * Z),
      fy + fy * Y * Y / Z2, -fy * X * Y / Z2, -fy * X / Z;
}

bool EdgeProjection::read(std::istream &in) { return false; }

bool EdgeProjection::write(std::ostream &out) const { return false; }

BA_G2o::BA_G2o() { std::cout << "construction BA_G2O" << std::endl; }

BA_G2o::~BA_G2o() { std::cout << "destruction BA_G2O" << std::endl; }

// void BA_G2o::initialization() {}

// void BA_G2o::run(){}

void BA_G2o::runG2o() {
  Sophus::SE3 se3_pose;
  bundleAdjustmentG2o(world_3d_points_, camera_2d_points_,
                      camera_intrinsics_mat_, se3_pose);
}

void BA_G2o::bundleAdjustmentG2o(
    const std::vector<Eigen::Vector3d,
                      Eigen::aligned_allocator<Eigen::Vector3d>> &points_3d,
    const std::vector<Eigen::Vector2d,
                      Eigen::aligned_allocator<Eigen::Vector2d>> &points_2d,
    const cv::Mat &K, Sophus::SE3 &pose) {

  // 1. 设置求解方式　LinearSolverDense dense cholesky分解
  Block::LinearSolverType *linear_solver =
      new g2o::LinearSolverDense<Block::PoseMatrixType>();

  // 2. set block solver
  Block *solver_ptr = new Block(linear_solver);

  // 3. set OptimizationAlgorithm based on gauss-newton 设置总求解器
  g2o::OptimizationAlgorithmGaussNewton *solver =
      new g2o::OptimizationAlgorithmGaussNewton(solver_ptr);

  // 4. create sparse optimizer 设置稀疏优化器
  g2o::SparseOptimizer optimizer;
  optimizer.setAlgorithm(solver);
  optimizer.setVerbose(true);

  // 5. 往图中添加顶点
  VertexPose *vertex_pose =
      new VertexPose(); //　添加相机位姿原点 camera vertex_pose
  vertex_pose->setId(0);
  vertex_pose->setEstimate(Sophus::SE3());
  optimizer.addVertex(vertex_pose);

  // K
  Eigen::Matrix3d K_eigen;
  K_eigen << K.at<double>(0, 0), K.at<double>(0, 1), K.at<double>(0, 2),
      K.at<double>(1, 0), K.at<double>(1, 1), K.at<double>(1, 2),
      K.at<double>(2, 0), K.at<double>(2, 1), K.at<double>(2, 2);

  // edges
  int index = 1;
  for (size_t i = 0; i < points_2d.size(); ++i) {
    auto p2d = points_2d[i];
    auto p3d = points_3d[i];
    EdgeProjection *edge = new EdgeProjection(p3d, K_eigen);
    edge->setId(index);
    edge->setVertex(0, vertex_pose); // 设置连接的顶点
    edge->setMeasurement(p2d);       // 观测值
    edge->setInformation(Eigen::Matrix2d::Identity());
    optimizer.addEdge(edge);
    index++;
  }

  optimizer.setVerbose(true);
  optimizer.initializeOptimization();
  optimizer.optimize(iteration_time_);

  std::cout << "pnp camera pose estimated by g2o =\n"
            << vertex_pose->estimate().matrix() << std::endl;
  pose = vertex_pose->estimate();
}