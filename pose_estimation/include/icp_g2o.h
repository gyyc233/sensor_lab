#ifndef __H_ICP_G2O_H__
#define __H_ICP_G2O_H__

#include "my_time.h"
#include "sophus/se3.h"
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/SVD>
#include <chrono>
#include <g2o/core/base_unary_edge.h>
#include <g2o/core/base_vertex.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/solvers/dense/linear_solver_dense.h>
#include <iostream>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>

namespace SensorLab {
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

class EdgeProjectXYZRGBDPoseOnly
    : public g2o::BaseUnaryEdge<3, Eigen::Vector3d, VertexPose> {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  EdgeProjectXYZRGBDPoseOnly(const Eigen::Vector3d &pos);

  // estimate error
  virtual void computeError() override;

  // estimate jacobian matrix
  virtual void linearizeOplus() override;

  virtual bool read(std::istream &in) override;

  virtual bool write(std::ostream &out) const override;

private:
  Eigen::Vector3d _pos3d;
};

class ICP_G2O {
public:
  void bundleAdjustment(const std::vector<cv::Point3f> &points_source,
                        const std::vector<cv::Point3f> &points_target,
                        cv::Mat &rotation, cv::Mat &translation);

private:
  CostMillisecond cost_;
};
} // namespace SensorLab

#endif
