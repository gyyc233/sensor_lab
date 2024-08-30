#include <g2o/core/base_binary_edge.h>
#include <g2o/core/base_vertex.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/core/robust_kernel_impl.h>
#include <g2o/solvers/dense/linear_solver_dense.h>
#include <iostream>

#include "slam_book_bal_problem.h"
#include "sophus/se3.h"

using namespace Sophus;
using namespace Eigen;
using namespace std;

/// 姿态和内参的结构
struct PoseAndIntrinsics {
  PoseAndIntrinsics() {}

  /// set from given data address
  // 显式地声明explicit，防止隐式转换
  explicit PoseAndIntrinsics(double *data_addr) {
    // BAL 数据集提供的旋转是李代数（旋转向量）
    rotation = SO3::exp(Vector3d(data_addr[0], data_addr[1], data_addr[2]));
    translation = Vector3d(data_addr[3], data_addr[4], data_addr[5]);
    focal = data_addr[6];
    k1 = data_addr[7];
    k2 = data_addr[8];
  }

  /// 将估计值放入内存
  void set_to(double *data_addr) {
    auto r = rotation.log(); // 李群转成李代数存储
    for (int i = 0; i < 3; ++i)
      data_addr[i] = r[i];
    for (int i = 0; i < 3; ++i)
      data_addr[i + 3] = translation[i];
    data_addr[6] = focal;
    data_addr[7] = k1;
    data_addr[8] = k2;
  }

  SO3 rotation; // 旋转矩阵
  Vector3d translation = Vector3d::Zero();
  double focal = 0;
  double k1 = 0, k2 = 0;
};

/// 定义顶点：相机的位姿
/// 位姿加相机内参的顶点，9维，前三维为so3，接下去为t, f, k1, k2
class VertexPoseAndIntrinsics : public g2o::BaseVertex<9, PoseAndIntrinsics> {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

  VertexPoseAndIntrinsics() {}

  // 估计值重置
  virtual void setToOriginImpl() override { _estimate = PoseAndIntrinsics(); }

  // 计算下一次估计值，xk+1 = xk + ∆x
  virtual void oplusImpl(const double *update) override {
    _estimate.rotation = SO3::exp(Vector3d(update[0], update[1], update[2])) *
                         _estimate.rotation;
    _estimate.translation += Vector3d(update[3], update[4], update[5]);
    _estimate.focal += update[6];
    _estimate.k1 += update[7];
    _estimate.k2 += update[8];
  }

  /// 根据估计值投影一个２d点
  Vector2d project(const Vector3d &point) {
    Vector3d pc = _estimate.rotation * point + _estimate.translation;
    pc = -pc / pc[2];
    double r2 = pc.squaredNorm();
    double distortion = 1.0 + r2 * (_estimate.k1 + _estimate.k2 * r2);
    return Vector2d(_estimate.focal * distortion * pc[0],
                    _estimate.focal * distortion * pc[1]);
  }

  virtual bool read(istream &in) { return false; }

  virtual bool write(ostream &out) const { return false; }
};

// 顶点：路标点位置[x,y,z]
class VertexPoint : public g2o::BaseVertex<3, Vector3d> {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

  VertexPoint() {}

  virtual void setToOriginImpl() override { _estimate = Vector3d(0, 0, 0); }

  virtual void oplusImpl(const double *update) override {
    _estimate += Vector3d(update[0], update[1], update[2]);
  }

  virtual bool read(istream &in) { return false; }

  virtual bool write(ostream &out) const { return false; }
};

// 定义二元边
// 边的维度为2，数据类型是Vecor2d，连接的两个顶点类型分别是相机和路标点　VertexPoseAndIntrinsics,
// VertexPoint
class EdgeProjection
    : public g2o::BaseBinaryEdge<2, Vector2d, VertexPoseAndIntrinsics,
                                 VertexPoint> {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

  virtual void computeError() override {
    auto v0 = (VertexPoseAndIntrinsics *)_vertices[0];
    auto v1 = (VertexPoint *)_vertices[1];
    auto proj = v0->project(v1->estimate()); // 计算投影值
    _error = proj - _measurement;            // 计算误差值
  }

  // use numeric derivatives
  // 这里没有声明雅克比的函数，所以是数值求导

  virtual bool read(istream &in) { return false; }

  virtual bool write(ostream &out) const { return false; }
};

void SolveBA(SlamBookBALProblem &bal_problem);

int main(int argc, char **argv) {
  SlamBookBALProblem bal_problem("./data/problem-49-7776-pre.txt");
  bal_problem.Normalize();
  bal_problem.Perturb(0.1, 0.5, 0.5);
  bal_problem.WriteToPLYFile("ba_g2o_test2_initial.ply");
  SolveBA(bal_problem);
  bal_problem.WriteToPLYFile("ba_g2o_test2_final.ply");

  return 0;
}

// clang-format off
void SolveBA(SlamBookBALProblem &bal_problem) {
  const int point_block_size = bal_problem.point_block_size();
  const int camera_block_size = bal_problem.camera_block_size();
  double *points = bal_problem.mutable_points();
  double *cameras = bal_problem.mutable_cameras();

  typedef g2o::BlockSolver<g2o::BlockSolverTraits<9, 3>>
      BlockSolverType; //　相机位姿和内参畸变等相机是９维，观测点是３维
  typedef g2o::LinearSolverDense<BlockSolverType::PoseMatrixType>
      LinearSolverType;

  BlockSolverType::LinearSolverType *linear_solver =
      new g2o::LinearSolverDense<BlockSolverType::PoseMatrixType>();

  // 2. set block solver
  BlockSolverType *solver_ptr = new BlockSolverType(linear_solver);
  g2o::OptimizationAlgorithmLevenberg *solver =
      new g2o::OptimizationAlgorithmLevenberg(solver_ptr);

  g2o::SparseOptimizer optimizer;
  optimizer.setAlgorithm(solver);
  optimizer.setVerbose(true);

  /// build g2o problem
  const double *observations = bal_problem.observations();

  // 将每个相机位姿加入顶点
  vector<VertexPoseAndIntrinsics *> vertex_pose_intrinsics;
  vector<VertexPoint *> vertex_points;
  for (int i = 0; i < bal_problem.num_cameras(); ++i) {
    VertexPoseAndIntrinsics *v = new VertexPoseAndIntrinsics();
    double *camera = cameras + camera_block_size * i;
    v->setId(i);
    v->setEstimate(PoseAndIntrinsics(camera));  // 设置顶点的估计初始值
    optimizer.addVertex(v); // 添加相机位姿顶点
    vertex_pose_intrinsics.push_back(v);
  }

  // 将每个路标点加入顶点
  for (int i = 0; i < bal_problem.num_points(); ++i) {
    VertexPoint *v = new VertexPoint();
    double *point = points + point_block_size * i;
    v->setId(i + bal_problem.num_cameras());
    v->setEstimate(Vector3d(point[0], point[1], point[2])); // 设置顶点的估计初始值
    // g2o在BA中需要手动设置待Marg边缘化的顶点
    v->setMarginalized(true); // 手动设置稀疏求解
    optimizer.addVertex(v); // 添加路标点顶点
    vertex_points.push_back(v);
  }

  // edge　每一个观测值都是一条边
  for (int i = 0; i < bal_problem.num_observations(); ++i) {
    EdgeProjection *edge = new EdgeProjection;
    // bal_problem.camera_index()[i]得到的是当前观测数据对应的相机位姿的序号，
    // 作为数组索引恰好得到vector容器中存放的相机位姿顶点的指针
    edge->setVertex(0, vertex_pose_intrinsics[bal_problem.camera_index()[i]]); // 设置这条边的第一个顶点
    // bal_problem.point_index()[i]得到的是当前观测数据对应的路标点的序号，
    // 作为数组索引恰好得到vector容器中存放的路标点顶点的指针
    edge->setVertex(1, vertex_points[bal_problem.point_index()[i]]);// 设置这条边的第二个顶点

    edge->setMeasurement(Vector2d(observations[2 * i + 0], observations[2 * i + 1])); // 设置观测值
    edge->setInformation(Matrix2d::Identity()); // 设置信息矩阵，这里设置成单位阵，表示各个观测数据的误差权重都相等
    edge->setRobustKernel(new g2o::RobustKernelHuber()); // 鲁邦核函数，防止噪声数据的过大影响
    optimizer.addEdge(edge);
  }

  optimizer.initializeOptimization();
  optimizer.optimize(40);

  // 程序执行到这里的时候，已经优化完毕了，为了后面
  // BLAproblem这个类把优化的结果存盘使用，所以还需要把这个结果保存
  // set to bal problem
  for (int i = 0; i < bal_problem.num_cameras(); ++i) {
    double *camera = cameras + camera_block_size * i; // 获得要存储的这组数据的指针
    auto vertex = vertex_pose_intrinsics[i]; // 获取顶点
    auto estimate = vertex->estimate(); // 顶点的估计值是顶点数据类型（自定义的对象）
    estimate.set_to(camera); // 调用自定义对象中存到内存中的函数，把优化后的数据，存到读取数据的内存中，相当于更新了内存值
  }
  for (int i = 0; i < bal_problem.num_points(); ++i) {
    double *point = points + point_block_size * i;
    auto vertex = vertex_points[i];
    for (int k = 0; k < 3; ++k) // 这里位姿顶点的数据类型是Vector3d，所以这里返回的估计值也是Vector3d
      point[k] = vertex->estimate()[k];
  }
}
// clang-format on
