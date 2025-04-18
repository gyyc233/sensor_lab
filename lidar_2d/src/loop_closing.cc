#ifdef ROS_CATKIN

#include "loop_closing.h"
#include "g2o_types.h"
#include "lidar_2d_utils.h"

#include <glog/logging.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>

#include <g2o/core/base_unary_edge.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/core/robust_kernel.h>
#include <g2o/core/robust_kernel_impl.h>
#include <g2o/core/sparse_optimizer.h>
#include <g2o/solvers/dense/linear_solver_dense.h>

namespace sad {
void LoopClosing::addNewSubmap(std::shared_ptr<Submap> submap) {
  submaps_.emplace(submap->getId(), submap);
  last_submap_id_ = submap->getId();
}

void LoopClosing::addFinishedSubmap(std::shared_ptr<Submap> submap) {
  auto mr_field = std::make_shared<MRLikelihoodField>();
  mr_field->setPose(submap->getPose());
  mr_field->setFieldImageFromOccuMap(
      submap->getOccupancyMap().getOccupancyGrid());
  submap_to_field_.emplace(submap, mr_field);
}

void LoopClosing::addNewFrame(std::shared_ptr<Frame> frame) {
  current_frame_ = frame;
  if (!detectLoopCandidates()) {
    return;
  }

  matchInHistorySubmaps();

  if (has_new_loops_) {
    optimize();
  }
}

bool LoopClosing::detectLoopCandidates() {
  // 要求当前帧与历史submap有一定间隔
  has_new_loops_ = false;
  if (last_submap_id_ < submap_gap_) {
    return false;
  }

  current_candidates_.clear();

  for (auto &sp : submaps_) {
    if ((last_submap_id_ - sp.first) <= submap_gap_) {
      // 不检查最近的几个submap
      continue;
    }

    // 如果这个submap和历史submap已经存在有效的关联，也忽略之
    // find(key)
    auto hist_iter = loop_constraints_.find(
        std::pair<size_t, size_t>(sp.first, last_submap_id_));
    if (hist_iter != loop_constraints_.end() && hist_iter->second.valid_) {
      continue;
    }

    Vec2d center = sp.second->getPose().translation();
    Vec2d frame_pos = current_frame_->pose_.translation();
    double dis = (center - frame_pos).norm();
    if (dis < candidate_distance_th_) {
      /// 如果这个frame离submap中心差距小于阈值，则检查
      LOG(INFO) << "taking " << current_frame_->keyframe_id_ << " with "
                << sp.first << ", last submap id: " << last_submap_id_;
      current_candidates_.emplace_back(sp.first);
    }
  }

  return !current_candidates_.empty();
}

void LoopClosing::matchInHistorySubmaps() {
  // 我们先把要检查的scan, pose和submap存到离线文件, 把mr match调完了再实际上线
  // current_frame_->Dump("./data/ch6/frame_" +
  // std::to_string(current_frame_->id_) + ".txt");

  for (const size_t &can : current_candidates_) {
    auto mr = submap_to_field_.at(submaps_[can]);
    mr->setSourceScan(current_frame_->scan_);

    auto submap = submaps_[can];
    SE2 pose_in_target_submap =
        submap->getPose().inverse() * current_frame_->pose_; // T_S1_C

    if (mr->alignG2O(pose_in_target_submap)) {
      // set constraints from current submap to target submap
      // T_S1_S2 = T_S1_C * T_C_W * T_W_S2
      SE2 T_this_cur = pose_in_target_submap * current_frame_->pose_.inverse() *
                       submaps_[last_submap_id_]->getPose();
      loop_constraints_.emplace(
          std::pair<size_t, size_t>(can, last_submap_id_),
          LoopConstraints(can, last_submap_id_, T_this_cur));
      LOG(INFO) << "adding loop from submap " << can << " to "
                << last_submap_id_;

      /// 可视化显示
      auto occu_image = submap->getOccupancyMap().getOccupancyGridBlackWhite();
      Visualize2DScan(current_frame_->scan_, pose_in_target_submap, occu_image,
                      Vec3b(0, 0, 255), 1000, 20.0, SE2());
      cv::putText(occu_image, "loop submap " + std::to_string(submap->getId()),
                  cv::Point2f(20, 20), cv::FONT_HERSHEY_COMPLEX, 0.5,
                  cv::Scalar(0, 255, 0));
      cv::putText(occu_image,
                  "keyframes " + std::to_string(submap->numFrames()),
                  cv::Point2f(20, 50), cv::FONT_HERSHEY_COMPLEX, 0.5,
                  cv::Scalar(0, 255, 0));
      cv::imshow("loop closure", occu_image);

      has_new_loops_ = true;
    }

    debug_fout_ << current_frame_->id_ << " " << can << " "
                << submaps_[can]->getPose().translation().x() << " "
                << submaps_[can]->getPose().translation().y() << " "
                << submaps_[can]->getPose().so2().log() << std::endl;
  }

  current_candidates_.clear();
}

void LoopClosing::optimize() {
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

  for (auto &sp : submaps_) {
    auto *v = new VertexSE2();
    v->setId(sp.first);
    v->setEstimate(sp.second->getPose());
    optimizer.addVertex(v);
  }

  /// 连续约束
  for (int i = 0; i < last_submap_id_; ++i) {
    auto first_submap = submaps_[i];
    auto next_submap = submaps_[i + 1];

    EdgeSE2 *e = new EdgeSE2();
    e->setVertex(0, optimizer.vertex(i));
    e->setVertex(1, optimizer.vertex(i + 1));
    e->setMeasurement(first_submap->getPose().inverse() *
                      next_submap->getPose());
    e->setInformation(Mat3d::Identity() * 1e4);

    optimizer.addEdge(e);
  }

  /// 回环约束
  std::map<std::pair<size_t, size_t>, EdgeSE2 *> loop_edges;
  for (auto &c : loop_constraints_) {
    if (!c.second.valid_) {
      continue;
    }

    auto first_submap = submaps_[c.first.first];
    auto second_submap = submaps_[c.first.second];

    EdgeSE2 *e = new EdgeSE2();
    e->setVertex(0, optimizer.vertex(first_submap->getId()));
    e->setVertex(1, optimizer.vertex(second_submap->getId()));
    e->setMeasurement(c.second.T12_);
    e->setInformation(Mat3d::Identity());

    auto rk = new g2o::RobustKernelCauchy;
    rk->setDelta(loop_rk_delta_);
    e->setRobustKernel(rk);

    optimizer.addEdge(e);
    loop_edges.emplace(c.first, e);
  }

  optimizer.setVerbose(true);
  optimizer.initializeOptimization();
  optimizer.optimize(10);

  // validate the loop constraints
  int inliers = 0;
  for (auto &ep : loop_edges) {
    if (ep.second->chi2() < loop_rk_delta_) {
      LOG(INFO) << "loop from " << ep.first.first << " to " << ep.first.second
                << " is correct, chi2: " << ep.second->chi2();
      ep.second->setRobustKernel(nullptr);
      loop_constraints_.at(ep.first).valid_ = true;
      inliers++;
    } else {
      ep.second->setLevel(1);
      LOG(INFO) << "loop from " << ep.first.first << " to " << ep.first.second
                << " is invalid, chi2: " << ep.second->chi2();
      loop_constraints_.at(ep.first).valid_ = false;
    }
  }

  optimizer.optimize(5);

  for (auto &sp : submaps_) {
    VertexSE2 *v = (VertexSE2 *)optimizer.vertex(sp.first);
    sp.second->setPose(v->estimate());

    // 更新所属的frame world pose
    sp.second->updateFramePoseWorld();
  }

  LOG(INFO) << "loop inliers: " << inliers << "/" << loop_constraints_.size();

  // 移除错误的匹配
  for (auto iter = loop_constraints_.begin();
       iter != loop_constraints_.end();) {
    if (!iter->second.valid_) {
      iter = loop_constraints_.erase(iter);
    } else {
      iter++;
    }
  }
}

} // namespace sad

#endif
