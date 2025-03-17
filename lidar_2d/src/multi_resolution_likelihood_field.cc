#ifdef ROS_CATKIN

#include "multi_resolution_likelihood_field.h"
#include "g2o_types.h"

#include <g2o/core/base_unary_edge.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/core/robust_kernel.h>
#include <g2o/core/robust_kernel_impl.h>
#include <g2o/core/sparse_optimizer.h>
#include <g2o/solvers/dense/linear_solver_dense.h>

#include <glog/logging.h>

namespace sad {
void MRLikelihoodField::buildModel() {
  int likelihood_range = 20; // 似然场模板范围

  field_ = {cv::Mat(125, 125, CV_32F, 30.0), cv::Mat(250, 250, CV_32F, 30.0),
            cv::Mat(500, 500, CV_32F, 30.0), cv::Mat(1000, 1000, CV_32F, 30.0)};

  for (int x = -likelihood_range; x < likelihood_range; x++) {
    for (int y = -likelihood_range; y < likelihood_range; y++) {
      model_.emplace_back(x, y, std::sqrt((x * x) + (y * y)));
    }
  }
}

void MRLikelihoodField::setFieldImageFromOccuMap(const cv::Mat &occu_map) {
  int boarder = 25;
  // default occu_map.rows and cols 1000
  for (int row = boarder; row < occu_map.rows; row++) {
    for (int col = boarder; col < occu_map.cols; col++) {

      if (occu_map.at<uchar>(col, row) < 127) {
        // 在该点生成一个model似然场，在每个level中都填入
        // 每个图像金字塔
        for (int l = 0; l < levels_; l++) {
          // 遍历似然场模板,在每个扫描点附近生成似然场
          for (auto &model_ptr : model_) {
            // 根据比例缩放到对应图像金字塔
            int cc = int(col * ratios_[l]);
            cc += model_ptr.dx_;

            int rr = int(row * ratios_[l]);
            rr += model_ptr.dy_;

            // 检查每个金字塔对应似然场函数值范围
            if (cc >= 0 && cc < field_[l].cols && rr >= 0 &&
                rr < field_[l].rows &&
                field_[l].at<float>(rr, cc) > model_ptr.residual_) {
              field_[l].at<float>(rr, cc) = model_ptr.residual_;
            }
          }
        }
      }
    }
  }
}

bool MRLikelihoodField::alignG2O(SE2 &init_pose) {
  num_inliers_.clear();
  inlier_ratio_.clear();

  for (int l = 0; l < levels_; ++l) {
    if (!alignInLevel(l, init_pose)) {
      return false;
    }
  }

  /// 成功匹配的话，打印一些信息
  for (int l = 0; l < levels_; ++l) {
    LOG(INFO) << "level " << l << " inliers: " << num_inliers_[l]
              << ", ratio: " << inlier_ratio_[l] << poses_[l];
  }

  return true;
}

std::vector<cv::Mat> MRLikelihoodField::getFieldImage() {
  std::vector<cv::Mat> images;
  for (int l = 0; l < levels_; ++l) {
    cv::Mat img(field_[l].rows, field_[l].cols, CV_8UC3);
    for (int x = 0; x < field_[l].cols; ++x) {
      for (int y = 0; y < field_[l].rows; ++y) {
        float r = field_[l].at<float>(y, x) * 255.0 / 30.0;
        img.at<cv::Vec3b>(y, x) = cv::Vec3b(uchar(r), uchar(r), uchar(r));
      }
    }

    images.emplace_back(img);
  }

  return images;
}

bool MRLikelihoodField::alignInLevel(int level, SE2 &init_pose) {
  // 这里直接搬运likelihood field g2o

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
  const double rk_delta[] = {0.2, 0.3, 0.6, 0.8};

  std::vector<EdgeSE2LikelihoodFiled *> edges;

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

    auto e =
        new EdgeSE2LikelihoodFiled(field_[level], r, angle, resolution_[level]);
    e->setVertex(0, v); // 设置链接的顶点，这里这个边更新后再次指向自己

    if (e->IsOutSide()) {
      delete e;
      continue;
    }

    e->setInformation(Eigen::Matrix<double, 1, 1>::Identity());

    // 设置huber核函数
    auto rk = new g2o::RobustKernelHuber;
    rk->setDelta(rk_delta[level]);
    e->setRobustKernel(rk);
    optimizer.addEdge(e);

    edges.emplace_back(e);
  }

  if (edges.empty()) {
    return false;
  }

  optimizer.setVerbose(false);
  optimizer.initializeOptimization();
  optimizer.optimize(10);

  /// 计算edges中有多少inlier
  int num_inliers =
      std::accumulate(edges.begin(), edges.end(), 0,
                      [&rk_delta, level](int num, EdgeSE2LikelihoodFiled *e) {
                        // g2o chi2 优化后的误差平方
                        if (e->level() == 0 && e->chi2() < rk_delta[level]) {
                          return num + 1; // inlier +1
                        }
                        return num;
                      });

  std::vector<double> chi2(edges.size());
  for (int i = 0; i < edges.size(); ++i) {
    chi2[i] = edges[i]->chi2();
  }

  std::sort(chi2.begin(), chi2.end());

  /// 要求inlier比例超过一定值
  /// 这个比较微妙，因为激光不是360的，这里大多数时候只能部分匹配上
  const float inlier_ratio_th = 0.4;
  float inlier_ratio = float(num_inliers) / edges.size();

  num_inliers_.emplace_back(num_inliers);
  inlier_ratio_.emplace_back(inlier_ratio);

  if (num_inliers > 100 && inlier_ratio > inlier_ratio_th) {
    init_pose = v->estimate();
    poses_.emplace_back(v->estimate());
    return true;
  } else {
    // LOG(INFO) << "rejected because ratio is not enough: " << inlier_ratio;
    return false;
  }
}

} // namespace sad

#endif
