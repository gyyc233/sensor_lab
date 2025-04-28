#ifndef FEATURE_TRACKER_PINHOLE_CAMERA_H
#define FEATURE_TRACKER_PINHOLE_CAMERA_H

#include <opencv2/core/core.hpp>
#include <string>

#include "camera.h"
#include "ceres/rotation.h"

namespace sensor_lab {

class PinholeCamera : public Camera {
public:
  class Parameters : public Camera::Parameters {

  public:
    Parameters();
    Parameters(const std::string &cameraName, int w, int h, double k1,
               double k2, double p1, double p2, double fx, double fy, double cx,
               double cy, double focal);

    double &k1(void);
    double &k2(void);
    double &p1(void);
    double &p2(void);
    double &fx(void);
    double &fy(void);
    double &cx(void);
    double &cy(void);
    double &focal(void);

    double xi(void) const;
    double k1(void) const;
    double k2(void) const;
    double p1(void) const;
    double p2(void) const;
    double fx(void) const;
    double fy(void) const;
    double cx(void) const;
    double cy(void) const;
    double focal(void) const;

    /// @brief
    /// @param params [k1,k2,p1,p2,fx,fy,cx,cy,focal]
    /// @return
    virtual bool setParams(const std::vector<double> &params);

    Parameters &operator=(const Parameters &other);
    friend std::ostream &operator<<(std::ostream &out,
                                    const Parameters &params);

  private:
    double k1_;
    double k2_;
    double p1_;
    double p2_;
    double fx_;
    double fy_;
    double cx_;
    double cy_;
    double focal_;
  };

  PinholeCamera();

  /**
   * \brief Constructor from the projection model parameters
   */
  PinholeCamera(const std::string &cameraName, int imageWidth, int imageHeight,
                double k1, double k2, double p1, double p2, double fx,
                double fy, double cx, double cy, double focal);

  PinholeCamera(const Parameters &params);

  Camera::ModelType modelType(void) const;
  const std::string &cameraName(void) const;
  int imageWidth(void) const;
  int imageHeight(void) const;

  /// @brief estimate camera intrinsics (使用张正友标定法)
  /// @param board_size
  /// @param object_points
  /// @param image_points
  void
  estimateIntrinsics(const cv::Size &board_size,
                     const std::vector<std::vector<cv::Point3f>> &object_points,
                     const std::vector<std::vector<cv::Point2f>> &image_points);

  void setParameters(const Parameters &parameters);

  // Lift points from the image plane to the
  // sphere　像素坐标[u,v]投影到单位球面坐标
  virtual void liftSphere(const Eigen::Vector2d &p, Eigen::Vector3d &P) const;
  //%output P

  // Lift points from the image plane to the projective
  // space　像素坐标转归一化相机平面坐标(z==1)
  void liftProjective(const Eigen::Vector2d &p, Eigen::Vector3d &P) const;
  //%output P

  // Projects 3D points to the image plane (Pi function) 相机坐标转图像坐标
  void spaceToPlane(const Eigen::Vector3d &P, Eigen::Vector2d &p) const;
  //%output p

  /// @brief 相机坐标(归一化平面)p_u经过畸变参数进行去畸变，再转换到像素坐标系下
  /// [u,v]
  /// @param p_u
  /// @param p
  void undistToPlane(const Eigen::Vector2d &p_u, Eigen::Vector2d &p) const;

  /// @brief 对归一化相机平面上的点进行去畸变
  /// @param p_u
  /// @param d_u
  void distortion(const Eigen::Vector2d &p_u, Eigen::Vector2d &d_u) const;

  /// @brief Apply distortion to input point (from the normalised plane) and
  /// calculate Jacobian
  /// @param p_u
  /// @param d_u
  /// @param J
  void distortion(const Eigen::Vector2d &p_u, Eigen::Vector2d &d_u,
                  Eigen::Matrix2d &J) const;

  const Parameters &getParameters(void) const;

  void initUndistortMap(cv::Mat &map1, cv::Mat &map2,
                        double fScale = 1.0) const;
  cv::Mat initUndistortRectifyMap(
      cv::Mat &map1, cv::Mat &map2, float fx = -1.0f, float fy = -1.0f,
      cv::Size image_size = cv::Size(0, 0), float cx = -1.0f, float cy = -1.0f,
      cv::Mat rmat = cv::Mat::eye(3, 3, CV_32F)) const;

private:
  Parameters parameters_;

  double inv_K11_, inv_K13_, inv_K22_,
      inv_K23_; // 用于计算[u,v] -> camera point
  bool no_distortion_;
};

typedef std::shared_ptr<PinholeCamera> PinholeCameraPtr;
typedef std::shared_ptr<const PinholeCamera> PinholeCameraConstPtr;

} // namespace sensor_lab

#endif
