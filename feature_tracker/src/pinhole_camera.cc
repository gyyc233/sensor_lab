#include "pinhole_camera.h"
#include <cmath>
#include <cstdio>
#include <eigen3/Eigen/Dense>
#include <iomanip>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/imgproc/imgproc.hpp>

namespace sensor_lab {
PinholeCamera::Parameters::Parameters()
    : Camera::Parameters(PINHOLE), k1_(0.0), k2_(0.0), p1_(0.0), p2_(0.0),
      fx_(0.0), fy_(0.0), cx_(0.0), cy_(0.0), focal_(0.0) {}

PinholeCamera::Parameters::Parameters(const std::string &cameraName, int w,
                                      int h, double k1, double k2, double p1,
                                      double p2, double fx, double fy,
                                      double cx, double cy, double focal)
    : Camera::Parameters(PINHOLE, cameraName, w, h), k1_(k1), k2_(k2), p1_(p1),
      p2_(p2), fx_(fx), fy_(fy), cx_(cx), cy_(cy), focal_(focal) {}

double &PinholeCamera::Parameters::k1(void) { return k1_; }

double &PinholeCamera::Parameters::k2(void) { return k2_; }

double &PinholeCamera::Parameters::p1(void) { return p1_; }

double &PinholeCamera::Parameters::p2(void) { return p2_; }

double &PinholeCamera::Parameters::fx(void) { return fx_; }

double &PinholeCamera::Parameters::fy(void) { return fy_; }

double &PinholeCamera::Parameters::cx(void) { return cx_; }

double &PinholeCamera::Parameters::cy(void) { return cy_; }

double &PinholeCamera::Parameters::focal(void) { return focal_; }

double PinholeCamera::Parameters::k1(void) const { return k1_; }

double PinholeCamera::Parameters::k2(void) const { return k2_; }

double PinholeCamera::Parameters::p1(void) const { return p1_; }

double PinholeCamera::Parameters::p2(void) const { return p2_; }

double PinholeCamera::Parameters::fx(void) const { return fx_; }

double PinholeCamera::Parameters::fy(void) const { return fy_; }

double PinholeCamera::Parameters::cx(void) const { return cx_; }

double PinholeCamera::Parameters::cy(void) const { return cy_; }

double PinholeCamera::Parameters::focal(void) const { return focal_; }

bool PinholeCamera::Parameters::setParams(const std::vector<double> &params) {
  k1_ = params[0];
  k2_ = params[1];
  p1_ = params[2];
  p2_ = params[3];

  fx_ = params[4];
  fy_ = params[5];
  cx_ = params[6];
  cy_ = params[7];
  focal_ = params[8];

  return true;
}

PinholeCamera::Parameters &PinholeCamera::Parameters::
operator=(const Parameters &other) {
  if (this != &other) {
    model_type_ = other.model_type_;
    camera_name_ = other.camera_name_;
    image_width_ = other.image_width_;
    image_height_ = other.image_height_;

    k1_ = other.k1_;
    k2_ = other.k2_;
    p1_ = other.p1_;
    p2_ = other.p2_;

    fx_ = other.fx_;
    fy_ = other.fy_;
    cx_ = other.cx_;
    cy_ = other.cy_;
    focal_ = other.focal_;
  }

  return *this;
}

std::ostream &operator<<(std::ostream &out,
                         const PinholeCamera::Parameters &params) {
  out << "Camera Parameters:" << std::endl;
  out << "    model_type "
      << "PINHOLE" << std::endl;
  out << "   camera_name " << params.camera_name_ << std::endl;
  out << "   image_width " << params.image_width_ << std::endl;
  out << "  image_height " << params.image_height_ << std::endl;

  // radial distortion: k1, k2
  // tangential distortion: p1, p2
  out << "Distortion Parameters" << std::endl;
  out << "            k1 " << params.k1_ << std::endl
      << "            k2 " << params.k2_ << std::endl
      << "            p1 " << params.p1_ << std::endl
      << "            p2 " << params.p2_ << std::endl;

  // projection: fx, fy, cx, cy
  out << "Projection Parameters" << std::endl;
  out << "            fx " << params.fx_ << std::endl
      << "            fy " << params.fy_ << std::endl
      << "            cx " << params.cx_ << std::endl
      << "            cy " << params.cy_ << std::endl
      << "      focal " << params.focal_ << std::endl;

  return out;
}

PinholeCamera::PinholeCamera()
    : inv_K11_(1.0), inv_K13_(0.0), inv_K22_(1.0), inv_K23_(0.0),
      no_distortion_(true) {}

PinholeCamera::PinholeCamera(const std::string &camera_name, int image_width,
                             int image_height, double k1, double k2, double p1,
                             double p2, double fx, double fy, double cx,
                             double cy, double focal)
    : parameters_(camera_name, image_width, image_height, k1, k2, p1, p2, fx,
                  fy, cx, cy, focal) {
  if ((parameters_.k1() == 0.0) && (parameters_.k2() == 0.0) &&
      (parameters_.p1() == 0.0) && (parameters_.p2() == 0.0)) {
    no_distortion_ = true;
  } else {
    no_distortion_ = false;
  }

  // Inverse camera projection matrix parameters
  inv_K11_ = 1.0 / parameters_.fx();
  inv_K13_ = -parameters_.cx() / parameters_.fx();
  inv_K22_ = 1.0 / parameters_.fy();
  inv_K23_ = -parameters_.cy() / parameters_.fy();
}

PinholeCamera::PinholeCamera(const PinholeCamera::Parameters &params)
    : parameters_(params) {
  if ((parameters_.k1() == 0.0) && (parameters_.k2() == 0.0) &&
      (parameters_.p1() == 0.0) && (parameters_.p2() == 0.0)) {
    no_distortion_ = true;
  } else {
    no_distortion_ = false;
  }

  // Inverse camera projection matrix parameters
  inv_K11_ = 1.0 / parameters_.fx();
  inv_K13_ = -parameters_.cx() / parameters_.fx();
  inv_K22_ = 1.0 / parameters_.fy();
  inv_K23_ = -parameters_.cy() / parameters_.fy();
}

Camera::ModelType PinholeCamera::modelType(void) const {
  return parameters_.modelType();
}

const std::string &PinholeCamera::cameraName(void) const {
  return parameters_.cameraName();
}

int PinholeCamera::imageWidth(void) const { return parameters_.imageWidth(); }

int PinholeCamera::imageHeight(void) const { return parameters_.imageHeight(); }

void PinholeCamera::estimateIntrinsics(
    const cv::Size &board_size,
    const std::vector<std::vector<cv::Point3f>> &object_points,
    const std::vector<std::vector<cv::Point2f>> &image_points) {
  // Z. Zhang, A Flexible New Technique for Camera Calibration, PAMI 2000
  Parameters params = getParameters();

  // 假设零畸变
  params.k1() = 0.0;
  params.k2() = 0.0;
  params.p1() = 0.0;
  params.p2() = 0.0;

  double cx = params.imageWidth() / 2.0;
  double cy = params.imageHeight() / 2.0;
  params.cx() = cx;
  params.cy() = cy;

  size_t num_images = image_points.size();

  cv::Mat A(num_images * 2, 2, CV_64F);
  cv::Mat b(num_images * 2, 1, CV_64F);

  // 构建线性方程，闭式求解
  for (size_t i = 0; i < num_images; ++i) {
    const std::vector<cv::Point3f> &object_point = object_points.at(i);

    std::vector<cv::Point2f> M(object_point.size());
    for (size_t j = 0; j < M.size(); ++j) {
      M.at(j) = cv::Point2f(object_point.at(j).x, object_point.at(j).y);
    }

    // 计算两平面之间的单应矩阵
    cv::Mat H = cv::findHomography(M, image_points.at(i));

    H.at<double>(0, 0) -= H.at<double>(2, 0) * cx;
    H.at<double>(0, 1) -= H.at<double>(2, 1) * cx;
    H.at<double>(0, 2) -= H.at<double>(2, 2) * cx;
    H.at<double>(1, 0) -= H.at<double>(2, 0) * cy;
    H.at<double>(1, 1) -= H.at<double>(2, 1) * cy;
    H.at<double>(1, 2) -= H.at<double>(2, 2) * cy;

    double h[3], v[3], d1[3], d2[3];
    double n[4] = {0, 0, 0, 0};

    for (int j = 0; j < 3; ++j) {
      double t0 = H.at<double>(j, 0);
      double t1 = H.at<double>(j, 1);
      h[j] = t0;
      v[j] = t1;
      d1[j] = (t0 + t1) * 0.5;
      d2[j] = (t0 - t1) * 0.5;
      n[0] += t0 * t0;
      n[1] += t1 * t1;
      n[2] += d1[j] * d1[j];
      n[3] += d2[j] * d2[j];
    }

    for (int j = 0; j < 4; ++j) {
      n[j] = 1.0 / sqrt(n[j]);
    }

    for (int j = 0; j < 3; ++j) {
      h[j] *= n[0];
      v[j] *= n[1];
      d1[j] *= n[2];
      d2[j] *= n[3];
    }

    A.at<double>(i * 2, 0) = h[0] * v[0];
    A.at<double>(i * 2, 1) = h[1] * v[1];
    A.at<double>(i * 2 + 1, 0) = d1[0] * d2[0];
    A.at<double>(i * 2 + 1, 1) = d1[1] * d2[1];
    b.at<double>(i * 2, 0) = -h[2] * v[2];
    b.at<double>(i * 2 + 1, 0) = -d1[2] * d2[2];
  }

  cv::Mat f(2, 1, CV_64F);
  // 求解Ａf=b,计算f
  cv::solve(A, b, f, cv::DECOMP_NORMAL | cv::DECOMP_LU);

  // 分解相机内参
  params.fx() = sqrt(fabs(1.0 / f.at<double>(0)));
  params.fy() = sqrt(fabs(1.0 / f.at<double>(1)));

  setParameters(params);
}

void PinholeCamera::setParameters(const PinholeCamera::Parameters &parameters) {
  parameters_ = parameters;

  if ((parameters_.k1() == 0.0) && (parameters_.k2() == 0.0) &&
      (parameters_.p1() == 0.0) && (parameters_.p2() == 0.0)) {
    no_distortion_ = true;
  } else {
    no_distortion_ = false;
  }

  inv_K11_ = 1.0 / parameters_.fx();
  inv_K13_ = -parameters_.cx() / parameters_.fx();
  inv_K22_ = 1.0 / parameters_.fy();
  inv_K23_ = -parameters_.cy() / parameters_.fy();
}

void PinholeCamera::liftSphere(const Eigen::Vector2d &p,
                               Eigen::Vector3d &P) const {
  liftProjective(p, P);
  P.normalize();
}

void PinholeCamera::liftProjective(const Eigen::Vector2d &p,
                                   Eigen::Vector3d &P) const {
  double mx_d, my_d, mx2_d, mxy_d, my2_d, mx_u, my_u;
  double rho2_d, rho4_d, radDist_d, Dx_d, Dy_d, inv_denom_d;
  // double lambda;

  // Lift points to normalised plane
  mx_d = inv_K11_ * p(0) + inv_K13_;
  my_d = inv_K22_ * p(1) + inv_K23_;

  if (no_distortion_) {
    mx_u = mx_d;
    my_u = my_d;
  } else {
    if (0) {
      double k1 = parameters_.k1();
      double k2 = parameters_.k2();
      double p1 = parameters_.p1();
      double p2 = parameters_.p2();

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

      mx_u = mx_d - inv_denom_d * Dx_d;
      my_u = my_d - inv_denom_d * Dy_d;
    } else {
      // Recursive distortion model
      int n = 8;
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
  }

  // Obtain a projective ray
  P << mx_u, my_u, 1.0;
}

void PinholeCamera::spaceToPlane(const Eigen::Vector3d &P,
                                 Eigen::Vector2d &p) const {
  Eigen::Vector2d p_u, p_d;

  // Project points to the normalised plane
  p_u << P(0) / P(2), P(1) / P(2);

  if (no_distortion_) {
    p_d = p_u;
  } else {
    // Apply distortion
    Eigen::Vector2d d_u;
    distortion(p_u, d_u);
    p_d = p_u + d_u;
  }

  // Apply generalised projection matrix
  p << parameters_.fx() * p_d(0) + parameters_.cx(),
      parameters_.fy() * p_d(1) + parameters_.cy();
}

void PinholeCamera::undistToPlane(const Eigen::Vector2d &p_u,
                                  Eigen::Vector2d &p) const {
  Eigen::Vector2d p_d;

  if (no_distortion_) {
    p_d = p_u;
  } else {
    // Apply distortion
    // 对相机坐标系坐标进行去畸变
    Eigen::Vector2d d_u;
    distortion(p_u, d_u);
    p_d = p_u + d_u;
  }

  // Apply generalised projection matrix
  // 转到像素坐标
  p << parameters_.fx() * p_d(0) + parameters_.cx(),
      parameters_.fy() * p_d(1) + parameters_.cy();
}

void PinholeCamera::distortion(const Eigen::Vector2d &p_u,
                               Eigen::Vector2d &d_u) const {
  double k1 = parameters_.k1();
  double k2 = parameters_.k2();
  double p1 = parameters_.p1();
  double p2 = parameters_.p2();

  double mx2_u, my2_u, mxy_u, rho2_u, rad_dist_u;

  mx2_u = p_u(0) * p_u(0);
  my2_u = p_u(1) * p_u(1);
  mxy_u = p_u(0) * p_u(1);
  rho2_u = mx2_u + my2_u;
  rad_dist_u = k1 * rho2_u + k2 * rho2_u * rho2_u;
  d_u << p_u(0) * rad_dist_u + 2.0 * p1 * mxy_u + p2 * (rho2_u + 2.0 * mx2_u),
      p_u(1) * rad_dist_u + 2.0 * p2 * mxy_u + p1 * (rho2_u + 2.0 * my2_u);
}

void PinholeCamera::distortion(const Eigen::Vector2d &p_u, Eigen::Vector2d &d_u,
                               Eigen::Matrix2d &J) const {
  double k1 = parameters_.k1();
  double k2 = parameters_.k2();
  double p1 = parameters_.p1();
  double p2 = parameters_.p2();

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

const PinholeCamera::Parameters &PinholeCamera::getParameters(void) const {
  return parameters_;
}

void PinholeCamera::initUndistortMap(cv::Mat &map1, cv::Mat &map2,
                                     double fScale) const {
  cv::Size image_size(parameters_.imageWidth(), parameters_.imageHeight());

  cv::Mat mapX = cv::Mat::zeros(image_size, CV_32F);
  cv::Mat mapY = cv::Mat::zeros(image_size, CV_32F);

  for (int v = 0; v < image_size.height; ++v) {
    for (int u = 0; u < image_size.width; ++u) {
      // 像素坐标转相机坐标
      double mx_u = inv_K11_ / fScale * u + inv_K13_ / fScale;
      double my_u = inv_K22_ / fScale * v + inv_K23_ / fScale;

      Eigen::Vector3d P;
      P << mx_u, my_u, 1.0;

      Eigen::Vector2d p;
      // 再转像素坐标 emm....
      spaceToPlane(P, p);

      mapX.at<float>(v, u) = p(0);
      mapY.at<float>(v, u) = p(1);
    }
  }
  cv::convertMaps(mapX, mapY, map1, map2, CV_32FC1, false);
}

cv::Mat PinholeCamera::initUndistortRectifyMap(cv::Mat &map1, cv::Mat &map2,
                                               float fx, float fy,
                                               cv::Size image_size, float cx,
                                               float cy, cv::Mat rmat) const {
  if (image_size == cv::Size(0, 0)) {
    image_size = cv::Size(parameters_.imageWidth(), parameters_.imageHeight());
  }

  cv::Mat mapX = cv::Mat::zeros(image_size.height, image_size.width, CV_32F);
  cv::Mat mapY = cv::Mat::zeros(image_size.height, image_size.width, CV_32F);

  Eigen::Matrix3f R, R_inv;
  cv::cv2eigen(rmat, R);
  R_inv = R.inverse();

  // assume no skew
  Eigen::Matrix3f K_rect;

  if (cx == -1.0f || cy == -1.0f) {
    K_rect << fx, 0, image_size.width / 2, 0, fy, image_size.height / 2, 0, 0,
        1;
  } else {
    K_rect << fx, 0, cx, 0, fy, cy, 0, 0, 1;
  }

  if (fx == -1.0f || fy == -1.0f) {
    K_rect(0, 0) = parameters_.fx();
    K_rect(1, 1) = parameters_.fy();
  }

  Eigen::Matrix3f K_rect_inv = K_rect.inverse();

  for (int v = 0; v < image_size.height; ++v) {
    for (int u = 0; u < image_size.width; ++u) {
      Eigen::Vector3f xo;
      xo << u, v, 1;

      Eigen::Vector3f uo = R_inv * K_rect_inv * xo;

      Eigen::Vector2d p;
      spaceToPlane(uo.cast<double>(), p);

      mapX.at<float>(v, u) = p(0);
      mapY.at<float>(v, u) = p(1);
    }
  }

  cv::convertMaps(mapX, mapY, map1, map2, CV_32FC1, false);

  cv::Mat K_rect_cv;
  cv::eigen2cv(K_rect, K_rect_cv);
  return K_rect_cv;
}

} // namespace sensor_lab
