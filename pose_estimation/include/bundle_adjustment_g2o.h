#ifndef __H_BUNDLE_ADJUSTMENT_G2O_H__
#define __H_BUNDLE_ADJUSTMENT_G2O_H__

#include "bundle_adjustment_gauss_newton.h"
#include "sophus/se3.h"
#include <Eigen/Core>
#include <g2o/core/base_unary_edge.h>
#include <g2o/core/base_vertex.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/g2o_core_api.h>
#include <g2o/core/optimization_algorithm_dogleg.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/core/solver.h>
#include <g2o/solvers/dense/linear_solver_dense.h>
#include <memory>
#include <opencv2/opencv.hpp>
#include <string>
#include <vector>

namespace SensorLab {

// set g2o
typedef g2o::BlockSolver<g2o::BlockSolverTraits<6, 3>>
    Block; // 表示pose 是6维，观测点是3维
typedef g2o::LinearSolverDense<Block::PoseMatrixType>
    LinearSolverType; // 线性求解器类型

// set g2o vertex; 顶点是李群位姿
class VertexPose : public g2o::BaseVertex<6, Sophus::SE3> {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

  /// @brief reset estimate value
  virtual void setToOriginImpl() override;

  /// @brief left multiplication on SE3
  /// @param update
  virtual void oplusImpl(const double *update) override;

  virtual bool read(std::istream &in);
  virtual bool write(std::ostream &out) const;
};

// 这里只优化相机位姿,只用了一元边
class EdgeProjection
    : public g2o::BaseUnaryEdge<2, Eigen::Vector2d, VertexPose> {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  EdgeProjection(const Eigen::Vector3d &pos, const Eigen::Matrix3d &K);

  // estimate error
  virtual void computeError() override;

  // estimate jacobian matrix
  virtual void linearizeOplus() override;

  virtual bool read(std::istream &in) override;

  virtual bool write(std::ostream &out) const override;

private:
  Eigen::Vector3d _pos3d;
  Eigen::Matrix3d _K; // camera intrinsics matrix
};

class BA_G2o : public BA_GaussNewton {
public:
  BA_G2o();

  ~BA_G2o();

  // void initialization() override;

  // void run() override;

  void runG2o();

  void bundleAdjustmentG2o(
      const std::vector<Eigen::Vector3d,
                        Eigen::aligned_allocator<Eigen::Vector3d>> &points_3d,
      const std::vector<Eigen::Vector2d,
                        Eigen::aligned_allocator<Eigen::Vector2d>> &points_2d,
      const cv::Mat &K, Sophus::SE3 &pose);

private:
  g2o::OptimizationAlgorithmLevenberg *solver;
};
} // namespace SensorLab
#endif
