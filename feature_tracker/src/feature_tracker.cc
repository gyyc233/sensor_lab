#include "feature_tracker.h"
#include <iostream>
namespace sensor_lab {

using namespace std;
using namespace Eigen;

FeatureTracker::FeatureTracker() {
  image_rows = 480;
  image_cols = 752;
}

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
  int n_max_cnt = 200 - static_cast<int>(forw_pts.size());
  if (n_max_cnt > 0) {
    if (mask.empty())
      cout << "mask is empty " << endl;
    if (mask.type() != CV_8UC1)
      cout << "mask type wrong " << endl;
    if (mask.size() != forw_img.size())
      cout << "wrong size " << endl;
    cv::goodFeaturesToTrack(forw_img, n_pts, 200 - forw_pts.size(), 0.01, 20,
                            mask);
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

void FeatureTracker::rejectWithF() {}

void FeatureTracker::liftProjective(const Eigen::Vector2d &p,
                                    Eigen::Vector3d &P) {
  double mx_d, my_d, mx2_d, mxy_d, my2_d, mx_u, my_u;
  double rho2_d, rho4_d, radDist_d, Dx_d, Dy_d, inv_denom_d;

  // Lift points to normalised plane
  mx_d = camera_K[0] * p(0) + camera_K[2];
  my_d = camera_K[1] * p(1) + camera_K[3];

  // 无畸变
  // mx_u = mx_d;
  // my_u = my_d;

  // 传统多项式畸变
  if (0) {
    double k1 = distortion_param[0];
    double k2 = distortion_param[1];
    double p1 = distortion_param[3];
    double p2 = distortion_param[4];

    // Apply inverse distortion model
    // proposed by Heikkila
    mx2_d = mx_d * mx_d;
    my2_d = my_d * my_d;
    mxy_d = mx_d * my_d;
    rho2_d = mx2_d + my2_d;
    rho4_d = rho2_d * rho2_d;
    radDist_d = k1 * rho2_d + k2 * rho4_d;
    Dx_d = mx_d * radDist_d + p2 * (rho2_d + 2 * mx2_d) + 2 * p1 * mxy_d;
    Dy_d = my_d * radDist_d + p1 * (rho2_d + 2 * my2_d) + 2 * p2 * mxy_d;
    inv_denom_d = 1 / (1 + 4 * k1 * rho2_d + 6 * k2 * rho4_d + 8 * p1 * my_d +
                       8 * p2 * mx_d);

    // 逆向畸变矫正，计算出归一化平面坐标
    mx_u = mx_d - inv_denom_d * Dx_d;
    my_u = my_d - inv_denom_d * Dy_d;
  } else {
    // 迭代畸变矫正
    int n = 6;
    Eigen::Vector2d d_u;
    distortion(Eigen::Vector2d(mx_d, my_d), d_u);
    // Approximate value
    mx_u = mx_d - d_u(0);
    my_u = my_d - d_u(1);

    for (int i = 1; i < n; ++i) {
      distortion(Eigen::Vector2d(mx_u, my_u), d_u);
      mx_u = mx_d - d_u(0);
      my_u = my_d - d_u(1);
    }
  }

  // Obtain a projective ray
  // double xi = mParameters.xi();
  double xi = 1.0;
  if (xi == 1.0) // 等距投影,作为深度
  {
    P << mx_u, my_u, (1.0 - mx_u * mx_u - my_u * my_u) / 2.0;
  } else // 折反射投影，作为深度
  {
    // Reuse variable
    rho2_d = mx_u * mx_u + my_u * my_u;
    P << mx_u, my_u,
        1.0 - xi * (rho2_d + 1.0) / (xi + sqrt(1.0 + (1.0 - xi * xi) * rho2_d));
  }
}

void FeatureTracker::setDistortionParams(const vector<double> &params) {
  distortion_param = params;
}

void FeatureTracker::setCameraKMatrix(const vector<double> &camera_k) {
  camera_K = camera_k;
}

void FeatureTracker::undistortionDistortion(const Eigen::Vector2d &p_u,
                                            Eigen::Vector2d &d_u,
                                            Eigen::Matrix2d &J) {
  double k1 = distortion_param[0];
  double k2 = distortion_param[1];
  double p1 = distortion_param[3];
  double p2 = distortion_param[4];

  double mx2_u, my2_u, mxy_u, rho2_u, rad_dist_u;

  mx2_u = p_u(0) * p_u(0);
  my2_u = p_u(1) * p_u(1);
  mxy_u = p_u(0) * p_u(1);
  rho2_u = mx2_u + my2_u;
  rad_dist_u = k1 * rho2_u + k2 * rho2_u * rho2_u;
  d_u << p_u(0) * rad_dist_u + 2.0 * p1 * mxy_u + p2 * (rho2_u + 2.0 * mx2_u),
      p_u(1) * rad_dist_u + 2.0 * p2 * mxy_u + p1 * (rho2_u + 2.0 * my2_u);

  double dxdmx = 1.0 + rad_dist_u + k1 * 2.0 * mx2_u +
                 k2 * rho2_u * 4.0 * mx2_u + 2.0 * p1 * p_u(1) +
                 6.0 * p2 * p_u(0);
  double dydmx = k1 * 2.0 * p_u(0) * p_u(1) +
                 k2 * 4.0 * rho2_u * p_u(0) * p_u(1) + p1 * 2.0 * p_u(0) +
                 2.0 * p2 * p_u(1);
  double dxdmy = dydmx;
  double dydmy = 1.0 + rad_dist_u + k1 * 2.0 * my2_u +
                 k2 * rho2_u * 4.0 * my2_u + 6.0 * p1 * p_u(1) +
                 2.0 * p2 * p_u(0);

  J << dxdmx, dxdmy, dydmx, dydmy;
}

void FeatureTracker::distortion(const Eigen::Vector2d &p_u,
                                Eigen::Vector2d &d_u) const {
  double k1 = distortion_param[0];
  double k2 = distortion_param[1];
  double p1 = distortion_param[3];
  double p2 = distortion_param[4];

  double mx2_u, my2_u, mxy_u, rho2_u, rad_dist_u;

  mx2_u = p_u(0) * p_u(0);
  my2_u = p_u(1) * p_u(1);
  mxy_u = p_u(0) * p_u(1);
  rho2_u = mx2_u + my2_u;
  rad_dist_u = k1 * rho2_u + k2 * rho2_u * rho2_u;
  d_u << p_u(0) * rad_dist_u + 2.0 * p1 * mxy_u + p2 * (rho2_u + 2.0 * mx2_u),
      p_u(1) * rad_dist_u + 2.0 * p2 * mxy_u + p1 * (rho2_u + 2.0 * my2_u);
}

void FeatureTracker::undistortedPoints() {
  cur_un_pts.clear();
  cur_un_pts_map.clear();
  // cv::undistortPoints(cur_pts, un_pts, K, cv::Mat());
  for (unsigned int i = 0; i < cur_pts.size(); i++) {
    Eigen::Vector2d a(cur_pts[i].x, cur_pts[i].y);
    Eigen::Vector3d b;
    liftProjective(a, b);
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

} // namespace sensor_lab
