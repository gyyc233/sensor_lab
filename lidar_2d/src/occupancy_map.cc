#ifdef ROS_CATKIN
#include "occupancy_map.h"
#include "eigen_type/eigen_types.h"
#include "math_utils.h"
#include <glog/logging.h>

namespace sad {
OccupancyMap::OccupancyMap() {
  buildModel();
  occupancy_grid_ = cv::Mat(image_size_, image_size_, CV_8U, 127);
}

void OccupancyMap::BuildModel() {
  for (int x = -model_size_; x <= model_size_; x++) {
    for (int y = -model_size_; y <= model_size_; y++) {
      Model2DPoint pt;
      pt.dx_ = x;
      pt.dy_ = y;
      pt.range_ = sqrt(x * x + y * y) * inv_resolution_;
      pt.angle_ = std::atan2(y, x);
      pt.angle_ =
          pt.angle_ > M_PI ? pt.angle_ - 2 * M_PI : pt.angle_; // limit in 2pi
      model_.push_back(pt);
    }
  }
}

} // namespace sad
#endif
