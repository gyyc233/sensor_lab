#ifdef ROS_CATKIN

#include "submap.h"
#include <glog/logging.h>

namespace sad {
void Submap::setOccuFromOtherSubmap(std::shared_ptr<Submap> other) {
  auto frames_in_other = other->getFrames();

  for (size_t i = 0; i < frames_in_other.size(); i++) {
    if (i > 9) {
      break;
    }
    occupancy_map_.emplace_back(frames_in_other[i]);
  }

  // 根据当前栅格生成似然场
  field_.setFieldImageFromOccuMap(occupancy_map_.getOccupancyGrid());
}
} // namespace sad

#endif
