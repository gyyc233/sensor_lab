#ifdef ROS_CATKIN
#include "frame.h"

#include <fstream>
#include <glog/logging.h>

namespace sad {

void Frame::dump(const std::string &filename) {
  std::ofstream fout(filename);
  fout << id_ << " " << keyframe_id_ << " " << timestamp_ << std::endl;
  fout << pose_.translation()[0] << " " << pose_.translation()[1] << " "
       << pose_.so2().log() << std::endl;
  fout << scan_->angle_min << " " << scan_->angle_max << " "
       << scan_->angle_increment << " " << scan_->range_min << " "
       << scan_->range_max << " " << scan_->ranges.size() << std::endl;
  for (auto &r : scan_->ranges) {
    fout << r << " ";
  }
  fout.close();
}

void Frame::load(const std::string &filename) {
  std::ifstream fin(filename);
  if (!fin) {
    LOG(ERROR) << "cannot load from " << filename;
    return;
  }

  fin >> id_ >> keyframe_id_ >> timestamp_ >> pose_.translation()[0] >>
      pose_.translation()[1];
  double theta = 0;
  fin >> theta;
  pose_.so2() = SO2::exp(theta);
  scan_.reset(new Scan2d);
  fin >> scan_->angle_min >> scan_->angle_max >> scan_->angle_increment >>
      scan_->range_min >> scan_->range_max;

  int range_size;
  fin >> range_size;
  for (int i = 0; i < range_size; ++i) {
    double r;
    fin >> r;
    scan_->ranges.emplace_back(r);
  }
}

} // namespace sad

#endif
