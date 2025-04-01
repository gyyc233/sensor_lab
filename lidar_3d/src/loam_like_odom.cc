#ifdef ROS_CATKIN
#include "loam_like_odom.h"
#include "feature_extraction.h"
#include "math_utils.h"
#include "navigation_and_mapping/lidar_utils.h"
#include <glog/logging.h>

namespace sad {

LoamLikeOdom::LoamLikeOdom(LoamLikeOdom::Options options)
    : options_(options), feature_extraction_(new FeatureExtraction),
      global_map_(new PointCloudType()) {
  if (options_.display_realtime_cloud_) {
    viewer_ = std::make_shared<PCLMapViewer>(0.1);
  }

  kdtree_edge_.SetEnableANN();
  kdtree_surf_.SetEnableANN();
}

LoamLikeOdom::~LoamLikeOdom() {}

void LoamLikeOdom::processPointCloud(FullCloudPtr full_cloud) {
  LOG(INFO) << "processing frame " << cnt_frame_++;
  // step 1. 提特征
  CloudPtr current_edge(new PointCloudType), current_surf(new PointCloudType);
  feature_extraction_->extract(full_cloud, current_edge, current_surf);
}

} // namespace sad

#endif
