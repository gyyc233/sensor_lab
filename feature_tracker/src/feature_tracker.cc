#include "feature_tracker.h"
#include <iostream>
namespace sensor_lab {

using namespace std;
using namespace Eigen;

FeatureTracker::FeatureTracker() { max_corners_count_ = 200; }

FeatureTracker::~FeatureTracker() {}

bool inBorder(const cv::Point2f &pt) {
  const int BORDER_SIZE = 1;
  int img_x = cvRound(pt.x);
  int img_y = cvRound(pt.y);
  return BORDER_SIZE <= img_x && img_x < 752 - BORDER_SIZE &&
         BORDER_SIZE <= img_y && img_y < 480 - BORDER_SIZE;
}

void FeatureTracker::readImage(const cv::Mat &_img, double _cur_time) {
  if (forw_img.empty()) {
    prev_img = cur_img = forw_img = _img;
  } else {
    forw_img = _img;
  }

  cur_time = _cur_time;

  forw_pts.clear();

  if (cur_pts.size() > 0) {
    vector<uchar> status;
    vector<float> err;
    // 对两帧各自的特征点 cur_pts forw_pts 进行稀疏光流计算
    cv::calcOpticalFlowPyrLK(cur_img, forw_img, cur_pts, forw_pts, status, err,
                             cv::Size(21, 21), 3);

    for (int i = 0; i < int(forw_pts.size()); i++)
      if (status[i] && !inBorder(forw_pts[i]))
        status[i] = 0;

    reduceVector(prev_pts, status);
    reduceVector(cur_pts, status);
    reduceVector(forw_pts, status);
    reduceVector(ids, status);
    reduceVector(cur_un_pts, status);
    reduceVector(track_cnt, status);
  }

  rejectWithF();
  setMask();
  int n_max_cnt = max_corners_count_ - static_cast<int>(forw_pts.size());
  if (n_max_cnt > 0) {
    if (mask.empty())
      cout << "mask is empty " << endl;
    if (mask.type() != CV_8UC1)
      cout << "mask type wrong " << endl;
    if (mask.size() != forw_img.size())
      cout << "wrong size " << endl;

    // 检测 shi-tomas 角点
    cv::goodFeaturesToTrack(
        forw_img, n_pts, max_corners_count_ - forw_pts.size(), 0.01, 20, mask);
  } else {
    n_pts.clear();
  }
  addPoints();

  prev_img = cur_img;
  prev_pts = cur_pts;
  prev_un_pts = cur_un_pts;
  cur_img = forw_img;
  cur_pts = forw_pts;
  undistortedPoints();
  prev_time = cur_time;
}

void FeatureTracker::addPoints() {
  for (auto &p : n_pts) {
    forw_pts.push_back(p);
    ids.push_back(-1);
    track_cnt.push_back(1);
  }
}

void FeatureTracker::setMask() {
  mask = cv::Mat(image_rows, image_cols, CV_8UC1, cv::Scalar(255));
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
      cv::circle(mask, it.second.first, 5, 0, -1);
    }
  }
}

void FeatureTracker::rejectWithF() {
  if (forw_pts.size() >= 8) {
    TicToc t_f;
    vector<cv::Point2f> un_cur_pts(cur_pts.size());
    vector<cv::Point2f> un_forw_pts(forw_pts.size());

    for (unsigned int i = 0; i < cur_pts.size(); i++) {
      // 分别将 cur_pts 与 forw_pts 中的对应点对转到像素坐标系
      Eigen::Vector3d tmp_p;
      camera_ptr_->liftProjective(Eigen::Vector2d(cur_pts[i].x, cur_pts[i].y),
                                  tmp_p);
      tmp_p.x() = camera_ptr_->getParameters().focal() * tmp_p.x() / tmp_p.z() +
                  camera_ptr_->imageWidth() / 2.0;
      tmp_p.y() = camera_ptr_->getParameters().focal() * tmp_p.y() / tmp_p.z() +
                  camera_ptr_->imageHeight() / 2.0;
      un_cur_pts[i] = cv::Point2f(tmp_p.x(), tmp_p.y());

      camera_ptr_->liftProjective(Eigen::Vector2d(forw_pts[i].x, forw_pts[i].y),
                                  tmp_p);
      tmp_p.x() = camera_ptr_->getParameters().focal() * tmp_p.x() / tmp_p.z() +
                  camera_ptr_->imageWidth() / 2.0;
      tmp_p.y() = camera_ptr_->getParameters().focal() * tmp_p.y() / tmp_p.z() +
                  camera_ptr_->imageHeight() / 2.0;
      un_forw_pts[i] = cv::Point2f(tmp_p.x(), tmp_p.y());
    }

    // 应用到对极约束
    vector<uchar> status;
    cv::findFundamentalMat(un_cur_pts, un_forw_pts, cv::FM_RANSAC, 1.0, 0.99,
                           status);
    int size_a = cur_pts.size();

    reduceVector(prev_pts, status);
    reduceVector(cur_pts, status);
    reduceVector(forw_pts, status);
    reduceVector(cur_un_pts, status);
    reduceVector(ids, status);
    reduceVector(track_cnt, status);

    double check_rate = 1.0 * forw_pts.size() / size_a;

    if (check_rate <= 0.9) {
      std::cout << "FM ransac: " << size_a << " -> " << forw_pts.size() << ": "
                << 1.0 * forw_pts.size() / size_a << std::endl;
      std::cout << "FM ransac costs: " << t_f.toc() << "ms" << std::endl;
    }
  }
}

void FeatureTracker::undistortedPoints() {
  cur_un_pts.clear();
  cur_un_pts_map.clear();
  // cv::undistortPoints(cur_pts, un_pts, K, cv::Mat());
  for (unsigned int i = 0; i < cur_pts.size(); i++) {
    Eigen::Vector2d a(cur_pts[i].x, cur_pts[i].y);
    Eigen::Vector3d b;
    camera_ptr_->liftProjective(a, b);
    cur_un_pts.push_back(cv::Point2f(b.x() / b.z(), b.y() / b.z()));
    cur_un_pts_map.insert(
        make_pair(ids[i], cv::Point2f(b.x() / b.z(), b.y() / b.z())));
    // printf("cur pts id %d %f %f", ids[i], cur_un_pts[i].x, cur_un_pts[i].y);
  }
  // caculate points velocity
  if (!prev_un_pts_map.empty()) {
    double dt = cur_time - prev_time;
    pts_velocity.clear();
    for (unsigned int i = 0; i < cur_un_pts.size(); i++) {
      if (ids[i] != -1) {
        std::map<int, cv::Point2f>::iterator it;
        it = prev_un_pts_map.find(ids[i]);
        if (it != prev_un_pts_map.end()) {
          double v_x = (cur_un_pts[i].x - it->second.x) / dt;
          double v_y = (cur_un_pts[i].y - it->second.y) / dt;
          pts_velocity.push_back(cv::Point2f(v_x, v_y));
        } else
          pts_velocity.push_back(cv::Point2f(0, 0));
      } else {
        pts_velocity.push_back(cv::Point2f(0, 0));
      }
    }
  } else {
    for (unsigned int i = 0; i < cur_pts.size(); i++) {
      pts_velocity.push_back(cv::Point2f(0, 0));
    }
  }
  prev_un_pts_map = cur_un_pts_map;
}

void FeatureTracker::reduceVector(vector<int> &v, vector<uchar> status) {
  int j = 0;
  for (int i = 0; i < int(v.size()); i++) {
    if (status[i])
      v[j++] = v[i]; // 保留状态为1的元素
  }
  v.resize(j);
}

void FeatureTracker::reduceVector(vector<cv::Point2f> &v,
                                  vector<uchar> status) {
  int j = 0;
  for (int i = 0; i < int(v.size()); i++)
    if (status[i])
      v[j++] = v[i];
  v.resize(j);
}

void FeatureTracker::setMaxCornersCount(int max_corners_count) {
  max_corners_count_ = max_corners_count;
}

void FeatureTracker::setCamera(sensor_lab::PinholeCameraPtr camera_ptr) {
  camera_ptr_ = camera_ptr;
  image_rows = camera_ptr_->imageHeight();
  image_cols = camera_ptr_->imageWidth();
}

} // namespace sensor_lab
