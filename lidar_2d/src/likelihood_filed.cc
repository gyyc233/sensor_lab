#ifdef ROS_CATKIN

#include "likelihood_filed.h"
#include <glog/logging.h>

using namespace sad {

  void LikelihoodField::BuildModel() {
    const int range = 20; // 生成多少个像素的模板
    for (int x = -range; x <= range; ++x) {
      for (int y = -range; y <= range; ++y) {
        model_.emplace_back(x, y, std::sqrt((x * x) + (y * y)));
      }
    }
  }

  void LikelihoodField::SetTargetScan(Scan2d::Ptr scan) {
    target_ = scan;

    // 在target点上生成场函数
    field_ = cv::Mat(1000, 1000, CV_32F, 30.0);

    for (size_t i = 0; i < scan->ranges.size(); ++i) {
      if (scan->ranges[i] < scan->range_min ||
          scan->ranges[i] > scan->range_max) {
        continue;
      }

      double real_angle = scan->angle_min + i * scan->angle_increment;
      double x = scan->ranges[i] * std::cos(real_angle) * resolution_ + 500;
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

  void LikelihoodField::SetSourceScan(Scan2d::Ptr scan) { source_ = scan; }

  bool LikelihoodField::AlignGaussNewton(SE2 & init_pose) {
    int iterations = 10;
    double cost = 0, lastCost = 0;
    SE2 current_pose = init_pose;
    const int min_effect_pts = 20; // 最小有效点数
    const int image_boarder = 20;  // 预留图像边界

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
        Vec2d pw =
            current_pose * Vec2d(r * std::cos(angle), r * std::sin(angle));

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
}
#endif