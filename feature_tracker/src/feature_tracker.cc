#include "feature_tracker.h"

namespace sensor_lab {

using namespace std;
using namespace Eigen;

FeatureTracker::FeatureTracker() {
  image_rows = 480;
  image_cols = 752;
}

FeatureTracker::~FeatureTracker() {}

void FeatureTracker::readImage(const cv::Mat &_img, double _cur_time) {
  if (forw_img.empty()) {
    prev_img = cur_img = forw_img = img;
  } else {
    forw_img = img;
  }

  forw_pts.clear();

  if (cur_pts.size() > 0) {
  }
}

void FeatureTracker::setMask() {
  mask = cv::Mat(image_rows, image_cols, , CV_8UC1, cv::Scalar(255));
  // prefer to keep features that are tracked for long time
  vector<pair<int, pair<cv::Point2f, int>>> cnt_pts_id;

  for (unsigned int i = 0; i < forw_pts.size(); i++)
    cnt_pts_id.push_back(
        make_pair(track_cnt[i], make_pair(forw_pts[i], ids[i])));

  // 按帧时间顺序排列
  sort(cnt_pts_id.begin(), cnt_pts_id.end(),
       [](const pair<int, pair<cv::Point2f, int>> &a,
          const pair<int, pair<cv::Point2f, int>> &b) {
         return a.first > b.first;
       });

  // 重新计算下列三个变量
  forw_pts.clear();
  ids.clear();
  track_cnt.clear();

  for (auto &it : cnt_pts_id) {
    if (mask.at<uchar>(it.second.first) == 255) {
      forw_pts.push_back(it.second.first);
      ids.push_back(it.second.second);
      track_cnt.push_back(it.first);
      cv::circle(mask, it.second.first, MIN_DIST, 0, -1);
    }
  }
}

void FeatureTracker::rejectWithF() {}

void FeatureTracker::liftProjective(const Eigen::Vector2d &p,
                                    Eigen::Vector3d &P) {}

void FeatureTracker::setDistortionParams(const vector<double> &params) {
  distortion_param = params;
}

} // namespace sensor_lab
