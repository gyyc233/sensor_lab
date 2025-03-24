#ifdef ROS_CATKIN
#include "gen_simu_data.h"
#include <opencv2/core/core.hpp>

namespace sad {
GenSimuData::GenSimuData() {}

GenSimuData::GenSimuData(Options options) : options_(options) {}

GenSimuData::~GenSimuData() {}

void GenSimuData::genTarget() {
  cv::RNG rng;

  // 生成taget
  target_.reset(new PointCloudType());

  for (int i = 0; i < options_.num_points_; ++i) {
    int num_face = rng.uniform(0, 6);
    // 先随机生成一点，分配到６个面上
    Vec3d pt(rng.uniform(-options_.length_, options_.length_),
             rng.uniform(-options_.width_, options_.width_),
             rng.uniform(-options_.height_, options_.height_));

    if (num_face == 0) {
      pt.z() = -options_.height_;
    } else if (num_face == 1) {
      pt.z() = options_.height_;
    } else if (num_face == 2) {
      pt.x() = -options_.length_;
    } else if (num_face == 3) {
      pt.x() = options_.length_;
    } else if (num_face == 4) {
      pt.y() = -options_.width_;
    } else if (num_face == 5) {
      pt.y() = options_.width_;
    }
    target_->points.emplace_back(ToPointType(pt));
  }

  target_->height = 1;
  target_->width = options_.num_points_;
}

void GenSimuData::genData() {
  genTarget();

  // generate pose and source
  cv::RNG rng;
  Sophus::SO3 rng_rot(rng.gaussian(options_.pose_rot_sigma_),
                      rng.gaussian(options_.pose_rot_sigma_),
                      rng.gaussian(options_.pose_rot_sigma_));

  Eigen::Vector3d rng_trans(rng.gaussian(options_.pose_trans_sigma_),
                            rng.gaussian(options_.pose_trans_sigma_),
                            rng.gaussian(options_.pose_trans_sigma_));

  gt_pose_ = Sophus::SE3(rng_rot, rng_trans);

  source_.reset(new PointCloudType);
  for (int i = 0; i < options_.num_points_; i++) {
    source_->points.emplace_back(
        ToPointType(gt_pose_ * ToVec3d(target_->points[i])));
  }
  source_->height = 1;
  source_->width = source_->size();
}

CloudPtr GenSimuData::getTarget() const { return target_; }

CloudPtr GenSimuData::getSource() const { return source_; }

SE3 GenSimuData::getPose() const { return gt_pose_; }

} // namespace sad
#endif
