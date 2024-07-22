#include <Eigen/Core>
#include <chrono>
#include <cmath>
#include <g2o/core/base_unary_edge.h>
#include <g2o/core/base_vertex.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/g2o_core_api.h>
#include <g2o/core/optimization_algorithm_dogleg.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/solvers/dense/linear_solver_dense.h>
#include <iostream>
#include <memory>
#include <opencv2/core/core.hpp>
#include <vector>

// 定义顶点，模板参数：优化变量维度和数据类型
class CurveFittingVertex : public g2o::BaseVertex<3, Eigen::Vector3d> {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW // eigen 内存对齐

      virtual void
      setToOriginImpl() // 重置，设定被优化变量的原始值
  {
    _estimate << 0, 0, 0;
  }

  // 待优化的变量类型是否对+=封闭
  virtual void oplusImpl(const double *update) // 更新
  {
    _estimate += Eigen::Vector3d(update); // update强制类型转换为Vector3d
  }

  // 存盘和读盘：留空
  virtual bool read(std::istream &in) { return true; }
  virtual bool write(std::ostream &out) const { return true; }
};

int main(int argc, char **argv) {
  double ar = 1.0, br = 2.0, cr = 1.0;  // 真实参数值
  double ae = 2.0, be = -1.0, ce = 5.0; // 估计参数值
  int N = 100;                          // 数据点
  double w_sigma = 1.0;                 // 噪声Sigma值
  double inv_sigma = 1.0 / w_sigma;
  cv::RNG rng; // OpenCV随机数产生器

  std::vector<double> x_data, y_data; // 数据
  for (int i = 0; i < N; i++) {
    double x = i / 100.0;
    x_data.push_back(x);
    y_data.push_back(exp(ar * x * x + br * x + cr) +
                     rng.gaussian(w_sigma * w_sigma));
  }

  // 1. create LinearSolver
  // 每个误差项优化变量维度为3，误差值维度为1
  typedef g2o::BlockSolver<g2o::BlockSolverTraits<3, 1>> Block;
  typedef g2o::LinearSolverDense<Block::PoseMatrixType>
      LinearSolverType; // 线性求解器类型
  // 求解方式：LinearSolverDense dense cholesky分解法
  Block::LinearSolverType *linear_solver =
      new g2o::LinearSolverDense<Block::PoseMatrixType>();
  // std::unique_ptr<Block::LinearSolverType> linearSolver ( new
  // g2o::LinearSolverDense<Block::PoseMatrixType>());

  // 2. create block solvers
  Block *solver_ptr = new Block(linear_solver);
  // Block* solver_ptr = new Block (
  // std::unique_ptr<Block::LinearSolverType>(linearSolver) );

  // 3. create optimization_algorithm_gauss_newton 总求解器
  g2o::OptimizationAlgorithmLevenberg *solver =
      new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
  // g2o::OptimizationAlgorithmLevenberg* solver = new
  // g2o::OptimizationAlgorithmLevenberg ( std::unique_ptr<Block>(solver_ptr));

  // 4. create sparse optimizer
  g2o::SparseOptimizer optimizer;
  optimizer.setAlgorithm(solver);
  optimizer.setVerbose(true);

  // 5. 定义模型顶点

  // 6. 往图(稀疏优化器)中添加顶点
  CurveFittingVertex *vertex = new CurveFittingVertex();
  vertex->setEstimate(Eigen::Vector3d(ae, be, ce)); // 设置初始值
  vertex->setId(0);
  optimizer.addVertex(vertex);
  return 0;
}
