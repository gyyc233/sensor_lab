#include "icp_g2o.h"

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

EdgeProjectXYZRGBDPoseOnly::EdgeProjectXYZRGBDPoseOnly(
    const Eigen::Vector3d &pos)
    : _pos3d(pos) {}

void EdgeProjectXYZRGBDPoseOnly::computeError() {
  const VertexPose *pose = static_cast<const VertexPose *>(_vertices[0]);
  _error = _measurement - pose->estimate() * _pos3d;
}

void EdgeProjectXYZRGBDPoseOnly::linearizeOplus() {
  const VertexPose *pose = static_cast<VertexPose *>(_vertices[0]);
  Sophus::SE3 T = pose->estimate();
  Eigen::Vector3d xyz_trans = T * _pos3d;
  _jacobianOplusXi.block<3, 3>(0, 0) = -Eigen::Matrix3d::Identity();
  _jacobianOplusXi.block<3, 3>(0, 3) = Sophus::SO3::hat(xyz_trans);

  //第二种雅可比矩阵定义方式
  // double x = xyz_trans[0];
  // double y = xyz_trans[1];
  // double z = xyz_trans[2];

  // _jacobianOplusXi(0,0) = -1;
  // _jacobianOplusXi(0,1) = 0;
  // _jacobianOplusXi(0,2) = 0;
  // _jacobianOplusXi(0,3) = 0;
  // _jacobianOplusXi(0,4) = -z;
  // _jacobianOplusXi(0,5) = y;

  // _jacobianOplusXi(1,0) = 0;
  // _jacobianOplusXi(1,1) = -1;
  // _jacobianOplusXi(1,2) = 0;
  // _jacobianOplusXi(1,3) = z;
  // _jacobianOplusXi(1,4) = 0;
  // _jacobianOplusXi(1,5) = -x;

  // _jacobianOplusXi(2,0) = 0;
  // _jacobianOplusXi(2,1) = 0;
  // _jacobianOplusXi(2,2) = -1;
  // _jacobianOplusXi(2,3) = -y;
  // _jacobianOplusXi(2,4) = x;
  // _jacobianOplusXi(2,5) = 0;
}

bool EdgeProjectXYZRGBDPoseOnly::read(std::istream &in) { return false; }

bool EdgeProjectXYZRGBDPoseOnly::write(std::ostream &out) const {
  return false;
}

void ICP_G2O::bundleAdjustment(const std::vector<cv::Point3f> &points_source,
                               const std::vector<cv::Point3f> &points_target,
                               cv::Mat &R, cv::Mat &t) {
  // set g2o
  typedef g2o::BlockSolver<g2o::BlockSolverX>
      Block; // g2o::BlockSolverTraits<p，l>，p是估计值的维度，l为误差维度;当你不想了解到底是多少维，可以使用typedef
             // g2o::BlockSolverX
  typedef g2o::LinearSolverDense<Block::PoseMatrixType>
      LinearSolverType; // 线性求解器类型

  // 1. 设置求解方式　LinearSolverDense dense cholesky分解
  Block::LinearSolverType *linear_solver =
      new g2o::LinearSolverDense<Block::PoseMatrixType>();

  // 2. set block solver
  Block *solver_ptr = new Block(linear_solver);

  // 3. set OptimizationAlgorithm based on gauss-newton 设置总求解器
  g2o::OptimizationAlgorithmLevenberg *solver =
      new g2o::OptimizationAlgorithmLevenberg(solver_ptr);

  // 4. create sparse optimizer 设置稀疏优化器
  g2o::SparseOptimizer optimizer;
  optimizer.setAlgorithm(solver);
  optimizer.setVerbose(true);

  // vertex
  VertexPose *pose = new VertexPose(); // camera pose
  pose->setId(0);
  pose->setEstimate(Sophus::SE3());
  optimizer.addVertex(pose);

  // edges
  for (size_t i = 0; i < points_source.size(); i++) {
    EdgeProjectXYZRGBDPoseOnly *edge =
        new EdgeProjectXYZRGBDPoseOnly(Eigen::Vector3d(
            points_target[i].x, points_target[i].y, points_target[i].z));
    edge->setVertex(0, pose);
    edge->setMeasurement(Eigen::Vector3d(points_source[i].x, points_source[i].y,
                                         points_source[i].z));
    edge->setInformation(Eigen::Matrix3d::Identity());
    optimizer.addEdge(edge);
  }

  optimizer.initializeOptimization();
  optimizer.optimize(10);

  std::cout << "after optimization:\n" << std::endl;
  std::cout << "T=\n" << pose->estimate().matrix() << std::endl;

  // convert to cv::Mat
  Eigen::Matrix3d R_ = pose->estimate().rotation_matrix();
  Eigen::Vector3d t_ = pose->estimate().translation();
  R = (cv::Mat_<double>(3, 3) << R_(0, 0), R_(0, 1), R_(0, 2), R_(1, 0),
       R_(1, 1), R_(1, 2), R_(2, 0), R_(2, 1), R_(2, 2));
  t = (cv::Mat_<double>(3, 1) << t_(0, 0), t_(1, 0), t_(2, 0));

  std::cout << "rotation:\n" << R << std::endl;
  std::cout << "translation:\n" << t << std::endl;
}
