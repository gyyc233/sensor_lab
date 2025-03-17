#ifdef ROS_CATKIN

#include "likelihood_field.h"
#include "g2o_types.h"
#include <glog/logging.h>

#include <g2o/core/base_unary_edge.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/core/robust_kernel.h>
#include <g2o/core/robust_kernel_impl.h>
#include <g2o/core/sparse_optimizer.h>
#include <g2o/solvers/dense/linear_solver_dense.h>

namespace sad {

void LikelihoodField::buildModel() {
  const int range = 20; // 生成多少个像素的模板
  for (int x = -range; x <= range; ++x) {
    for (int y = -range; y <= range; ++y) {
      model_.emplace_back(x, y, std::sqrt((x * x) + (y * y)));
    }
  }
}

void LikelihoodField::setTargetScan(Scan2d::Ptr scan) {
  target_ = scan;

  // 在target点上生成场函数
  field_ = cv::Mat(1000, 1000, CV_32F, 30.0);

  for (size_t i = 0; i < scan->ranges.size(); ++i) {
    if (scan->ranges[i] < scan->range_min ||
        scan->ranges[i] > scan->range_max) {
      continue;
    }

    double real_angle = scan->angle_min + i * scan->angle_increment;
    double x = scan->ranges[i] * std::cos(real_angle) * resolution_ +
               500; // 将扫描数据偏移到图像中心
    double y = scan->ranges[i] * std::sin(real_angle) * resolution_ + 500;

    // 在每个(x,y)附近填入场函数
    for (auto &model_pt : model_) {
      int xx = int(x + model_pt.dx_);
      int yy = int(y + model_pt.dy_);

      // check xx yy
      if (xx >= 0 && xx < field_.cols && yy >= 0 && yy < field_.rows &&
          field_.at<float>(yy, xx) > model_pt.residual_) {
        field_.at<float>(yy, xx) = model_pt.residual_;
      }
    }
  }
}

void LikelihoodField::setSourceScan(Scan2d::Ptr scan) { source_ = scan; }

void LikelihoodField::setFieldImageFromOccuMap(const cv::Mat &occu_map) {
  const int boarder = 25;
  field_ = cv::Mat(1000, 1000, CV_32F, 30.0);

  for (int x = boarder; x < occu_map.cols - boarder; ++x) {
    for (int y = boarder; y < occu_map.rows - boarder; ++y) {
      if (occu_map.at<uchar>(y, x) < 127) {
        // 在该被占据点生成一个model
        for (auto &model_pt : model_) {
          int xx = int(x + model_pt.dx_);
          int yy = int(y + model_pt.dy_);
          if (xx >= 0 && xx < field_.cols && yy >= 0 && yy < field_.rows &&
              field_.at<float>(yy, xx) > model_pt.residual_) {
            field_.at<float>(yy, xx) = model_pt.residual_;
          }
        }
      }
    }
  }
}

bool LikelihoodField::alignGaussNewton(SE2 &init_pose) {
  int iterations = 10;
  double cost = 0, lastCost = 0;
  SE2 current_pose = init_pose;
  const int min_effect_pts = 20; // 最小有效点数
  const int image_boarder = 20; // 预留图像边界(似然场图像尺寸 980*980)

  has_outside_pts_ = false;
  for (int iter = 0; iter < iterations; ++iter) {
    Mat3d H = Mat3d::Zero();
    Vec3d b = Vec3d::Zero();
    cost = 0;

    int effective_num = 0; // 有效点数

    // 遍历source
    for (size_t i = 0; i < source_->ranges.size(); ++i) {
      float r = source_->ranges[i];
      if (r < source_->range_min || r > source_->range_max) {
        continue;
      }

      float angle = source_->angle_min + i * source_->angle_increment;
      if (angle < source_->angle_min + 30 * M_PI / 180.0 ||
          angle > source_->angle_max - 30 * M_PI / 180.0) {
        continue;
      }

      float theta = current_pose.so2().log();
      // 计算扫描数据的世界坐标
      Vec2d pw = current_pose * Vec2d(r * std::cos(angle), r * std::sin(angle));

      // 在field中的图像坐标
      Vec2i pf = (pw * resolution_ + Vec2d(500, 500)).cast<int>();

      if (pf[0] >= image_boarder && pf[0] < field_.cols - image_boarder &&
          pf[1] >= image_boarder && pf[1] < field_.rows - image_boarder) {
        effective_num++;

        // 图像梯度
        float dx = 0.5 * (field_.at<float>(pf[1], pf[0] + 1) -
                          field_.at<float>(pf[1], pf[0] - 1));
        float dy = 0.5 * (field_.at<float>(pf[1] + 1, pf[0]) -
                          field_.at<float>(pf[1] - 1, pf[0]));

        Vec3d J;
        J << resolution_ * dx, resolution_ * dy,
            -resolution_ * dx * r * std::sin(angle + theta) +
                resolution_ * dy * r * std::cos(angle + theta);
        H += J * J.transpose();

        // 残差直接用该点似然场函数的值表示
        float e = field_.at<float>(pf[1], pf[0]);
        b += -J * e;

        cost += e * e;
      } else {
        has_outside_pts_ = true;
      }
    }

    if (effective_num < min_effect_pts) {
      return false;
    }

    // solve for dx
    Vec3d dx = H.ldlt().solve(b);
    if (isnan(dx[0])) {
      break;
    }

    cost /= effective_num;
    if (iter > 0 && cost >= lastCost) {
      break;
    }

    LOG(INFO) << "iter " << iter << " cost = " << cost
              << ", effect num: " << effective_num;

    current_pose.translation() += dx.head<2>();
    current_pose.so2() = current_pose.so2() * SO2::exp(dx[2]);
    lastCost = cost;
  }

  init_pose = current_pose;
  return true;
}

bool LikelihoodField::alignG2O(SE2 &init_pose) {
  // 1. create LinearSolver
  // 每个误差项优化变量维度为3，误差值维度为1
  typedef g2o::BlockSolver<g2o::BlockSolverTraits<3, 1>> Block;
  typedef g2o::LinearSolverDense<Block::PoseMatrixType>
      LinearSolverType; // 线性求解器类型
  // 求解方式：LinearSolverDense dense cholesky分解法
  Block::LinearSolverType *linear_solver =
      new g2o::LinearSolverDense<Block::PoseMatrixType>();

  // 2. create block solvers
  Block *solver_ptr = new Block(linear_solver);

  // 3. create optimization_algorithm_gauss_newton 总求解器
  g2o::OptimizationAlgorithmLevenberg *solver =
      new g2o::OptimizationAlgorithmLevenberg(solver_ptr);

  // 4. create sparse optimizer
  g2o::SparseOptimizer optimizer;
  optimizer.setAlgorithm(solver);
  optimizer.setVerbose(true); // 打开调试输出

  auto *v = new VertexSE2();
  v->setId(0);
  v->setEstimate(init_pose);
  optimizer.addVertex(v);

  const double range_th = 15.0; // 不考虑太远的scan，不准
  const double rk_delta = 0.8;
  has_outside_pts_ = false;

  // 遍历source
  for (size_t i = 0; i < source_->ranges.size(); ++i) {
    float r = source_->ranges[i];
    if (r < source_->range_min || r > source_->range_max) {
      continue;
    }

    if (r > range_th) {
      continue;
    }

    float angle = source_->angle_min + i * source_->angle_increment;
    if (angle < source_->angle_min + 30 * M_PI / 180.0 ||
        angle > source_->angle_max - 30 * M_PI / 180.0) {
      continue;
    }

    // 创建似然场边
    auto e = new EdgeSE2LikelihoodFiled(field_, r, angle, resolution_);
    e->setVertex(0, v); // 设置链接的顶点，这里这个边更新后再次指向自己

    if (e->IsOutSide()) {
      has_outside_pts_ = true;
      delete e;
      continue;
    }

    // 设置信息矩阵
    e->setInformation(Eigen::Matrix<double, 1, 1>::Identity());

    // 设置huber核函数
    auto rk = new g2o::RobustKernelHuber;
    rk->setDelta(rk_delta);
    e->setRobustKernel(rk);
    optimizer.addEdge(e);
  }

  optimizer.initializeOptimization();
  optimizer.optimize(10);

  init_pose = v->estimate();
  pose_ = v->estimate();
  return true;
}

cv::Mat LikelihoodField::getFieldImage() {
  cv::Mat image(field_.rows, field_.cols, CV_8UC3);
  for (int x = 0; x < field_.cols; x++) {
    for (int y = 0; y < field_.rows; y++) {
      float r = field_.at<float>(y, x) * 255.0 / 30.0;
      image.at<cv::Vec3b>(y, x) = cv::Vec3b(uchar(r), uchar(r), uchar(r));
    }
  }
  return image;
}

} // namespace sad
#endif
