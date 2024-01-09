#include "cubic_spline/cubic_spline.h"
#include <glog/logging.h>

namespace Algorithm {

CubicSplineOperator::CubicSplineOperator() {
  max_x_ = 0;
  min_x_ = 0;
}

CubicSplineOperator::~CubicSplineOperator() {}

int CubicSplineOperator::setSamplePoints(const std::vector<double> &val_x,
                                         const std::vector<double> &val_y) {
  input_points_.clear();
  if (val_x.size() != val_y.size()) {
    LOG(ERROR) << "failed to sizes aligned";
    return -1;
  }

  input_points_.resize(val_x.size());
  if (val_x.size() <= 3) {
    LOG(ERROR) << "failed to set points, need more than 3 points";
    return -1;
  }

  min_x_ = *std::min_element(val_x.begin(), val_x.end());
  max_x_ = *std::max_element(val_x.begin(), val_x.end());

  for (size_t i = 0; i < val_x.size(); i++) {
    input_points_[i] =
        cv::Point2f(val_x[static_cast<int>(i)], val_y[static_cast<int>(i)]);
  }

  return 0;
}

int CubicSplineOperator::cubicSplineNatural() {
  int n = input_points_.size();
  a_ = cv::Mat::zeros(n - 1, 1, CV_32FC1);
  b_ = cv::Mat::zeros(n - 1, 1, CV_32FC1);
  c_ = cv::Mat::zeros(n - 1, 1, CV_32FC1);
  d_ = cv::Mat::zeros(n - 1, 1, CV_32FC1);
  cv::Mat dx = cv::Mat::zeros(n - 1, 1, CV_32FC1);
  cv::Mat dy = cv::Mat::zeros(n - 1, 1, CV_32FC1);

  for (int i = 0; i < n - 1; i++) {
    // calculate a, dx, dy
    a_.at<float>(i, 0) = input_points_[i].y;
    dx.at<float>(i, 0) = (input_points_[i + 1].x - input_points_[i].x);
    dy.at<float>(i, 0) = (input_points_[i + 1].y - input_points_[i].y);
  }

  // step 1. built H and d
  // H*m=D
  cv::Mat H = cv::Mat::zeros(n, n, CV_32FC1);
  cv::Mat D = cv::Mat::zeros(n, 1, CV_32FC1);
  H.at<float>(0, 0) = 1;
  H.at<float>(n - 1, n - 1) = 1;

  for (int i = 1; i <= n - 2; i++) {
    H.at<float>(i, i - 1) = dx.at<float>(i - 1, 0);
    H.at<float>(i, i) = 2 * (dx.at<float>(i - 1, 0) + dx.at<float>(i, 0));
    H.at<float>(i, i + 1) = dx.at<float>(i, 0);
    D.at<float>(i, 0) = 6 * (dy.at<float>(i, 0) / dx.at<float>(i, 0) -
                             dy.at<float>(i - 1, 0) / dx.at<float>(i - 1, 0));
  }

  // step 2. estimate m
  // step 3. estimate b, c, d
  cv::Mat m;
  estimateCoefMatrix(H, D, dx, dy, m);

  return 0;
}

void CubicSplineOperator::estimateCoefMatrix(const cv::Mat &H, const cv::Mat &D,
                                             const cv::Mat &dx,
                                             const cv::Mat &dy, cv::Mat &m) {
  int n = H.rows;
  m = cv::Mat::zeros(H.rows, 1, CV_32FC1);
  m = H.inv() * D;

  cv::Mat t1 = cv::Mat::zeros(n - 1, 1, CV_32FC1);
  cv::Mat t2 = cv::Mat::zeros(n - 1, 1, CV_32FC1);
  cv::Mat t3 = cv::Mat::zeros(n - 1, 1, CV_32FC1);
  for (int i = 0; i < n - 1; i++) {
    b_.at<float>(i, 0) = dy.at<float>(i, 0) / dx.at<float>(i, 0) -
                         (float)1 / 2 * dx.at<float>(i, 0) * m.at<float>(i, 0) -
                         (float)1 / 6 * dx.at<float>(i, 0) *
                             (m.at<float>(i + 1, 0) - m.at<float>(i, 0));

    t1.at<float>(i, 0) = dy.at<float>(i, 0) / dx.at<float>(i, 0);
    t2.at<float>(i, 0) = (float)-1 / 2 * dx.at<float>(i, 0) * m.at<float>(i, 0);
    t3.at<float>(i, 0) = (float)-1 / 6 * dx.at<float>(i, 0) *
                         (m.at<float>(i + 1, 0) - m.at<float>(i, 0));

    c_.at<float>(i, 0) = m.at<float>(i, 0) * 0.5f;
    d_.at<float>(i, 0) =
        (m.at<float>(i + 1, 0) - m.at<float>(i, 0)) / (6 * dx.at<float>(i, 0));
  }

  DLOG(INFO) << "estimate coefficient finish";
}

int CubicSplineOperator::cubicSplineFit(const std::vector<double> &vals,
                                        std::vector<double> &predicated_vals) {
  predicated_vals.resize(vals.size());

  int n = input_points_.size();
  for (size_t i = 0; i < vals.size(); i++) {

    int index = 0;
    for (int j = 0; j < n - 1; j++) {
      if (vals[i] >= input_points_[n - 1].x) {
        index = n - 2;
        break;
      } else if (vals[i] >= input_points_[j].x &&
                 vals[i] < input_points_[j + 1].x) {
        index = j;
        break;
      } else if (vals[i] < input_points_[0].x) {
        index = 0;
        break;
      }
    }

    float yn = a_.at<float>(index, 0) +
               b_.at<float>(index, 0) * (vals[i] - input_points_[index].x) +
               c_.at<float>(index, 0) * (vals[i] - input_points_[index].x) *
                   (vals[i] - input_points_[index].x) +
               d_.at<float>(index, 0) * (vals[i] - input_points_[index].x) *
                   (vals[i] - input_points_[index].x) *
                   (vals[i] - input_points_[index].x);

    predicated_vals[i] = yn;
    LOG(INFO) << "predicated vals: " << predicated_vals[i];
  }

  return 0;
}

void CubicSplineOperator::checkData(const std::vector<double> &vals,
                                    std::vector<double> &check_vals) {
  check_vals = vals;
  for (size_t i = 0; i < vals.size(); i++) {
    if (check_vals[i] < min_x_) {
      check_vals[i] = min_x_;
      continue;
    }
    if (check_vals[i] > max_x_) {
      check_vals[i] = max_x_;
      continue;
    }
  }
}

}; // namespace Algorithm