#ifndef FEATURE_TRACKER_CAMERA_H
#define FEATURE_TRACKER_CAMERA_H

#include <eigen3/Eigen/Dense>
#include <memory>
#include <opencv2/core/core.hpp>
#include <vector>

namespace sensor_lab {
class Camera {
public
  ;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  enum ModelType { KANNALA_BRANDT, MEI, PINHOLE, SCARAMUZZA };

  class Parameters {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    Parameters(ModelType modelType);

    Parameters(ModelType modelType, const std::string &cameraName, int w,
               int h);

    ModelType &modelType(void);
    std::string &cameraName(void);
    int &imageWidth(void);
    int &imageHeight(void);

    ModelType modelType(void) const;
    const std::string &cameraName(void) const;
    int imageWidth(void) const;
    int imageHeight(void) const;

    int nIntrinsics(void) const;

    virtual bool readFromYamlFile(const std::string &filename) = 0;
    virtual void writeToYamlFile(const std::string &filename) const = 0;

  protected:
    ModelType model_type_;
    int number_intrinsics_;
    std::string camera_name_;
    int image_width_;
    int image_height_;
  };

  virtual ModelType modelType(void) const = 0;
  virtual const std::string &cameraName(void) const = 0;
  virtual int imageWidth(void) const = 0;
  virtual int imageHeight(void) const = 0;

  virtual cv::Mat &mask(void);
  virtual const cv::Mat &mask(void) const;

  /// @brief estimate camera intrinsics
  /// @param boardSize
  /// @param objectPoints camera coordinate points
  /// @param imagePoints image coordinate [u,v]
  virtual void estimateIntrinsics(
      const cv::Size &boardSize,
      const std::vector<std::vector<cv::Point3f>> &objectPoints,
      const std::vector<std::vector<cv::Point2f>> &imagePoints) = 0;

  /// @brief estimate camera extrinsics
  /// @param objectPoints world coordinate points
  /// @param imagePoints image coordinate [u,v]
  /// @param r_vec rotation
  /// @param t_vec translation
  virtual void estimateExtrinsics(const std::vector<cv::Point3f> &objectPoints,
                                  const std::vector<cv::Point2f> &imagePoints,
                                  cv::Mat &r_vec, cv::Mat &t_vec) const;

  // Lift points from the image plane to the sphere　将点从归一化平面转到球面
  virtual void liftSphere(const Eigen::Vector2d &p,
                          Eigen::Vector3d &P) const = 0;
  //%output P

  // Lift points from the image plane to the projective
  // space　点从归一化平面转到投影空间
  virtual void liftProjective(const Eigen::Vector2d &p,
                              Eigen::Vector3d &P) const = 0;
  //%output P

  // Projects 3D points to the image plane (Pi
  // function)　将3d点投射到图像归一化平面
  virtual void spaceToPlane(const Eigen::Vector3d &P,
                            Eigen::Vector2d &p) const = 0;
  //%output p

  // Projects 3D points to the image plane (Pi function)
  // and calculates jacobian
  // virtual void spaceToPlane(const Eigen::Vector3d& P, Eigen::Vector2d& p,
  //                          Eigen::Matrix<double,2,3>& J) const = 0;
  //%output p
  //%output J

  virtual void undistToPlane(const Eigen::Vector2d &p_u,
                             Eigen::Vector2d &p) const = 0;
  //%output p

  // virtual void initUndistortMap(cv::Mat& map1, cv::Mat& map2, double fScale
  // = 1.0) const = 0;
  virtual cv::Mat
  initUndistortRectifyMap(cv::Mat &map1, cv::Mat &map2, float fx = -1.0f,
                          float fy = -1.0f, cv::Size imageSize = cv::Size(0, 0),
                          float cx = -1.0f, float cy = -1.0f,
                          cv::Mat rmat = cv::Mat::eye(3, 3, CV_32F)) const = 0;

  virtual int parameterCount(void) const = 0;

  virtual void readParameters(const std::vector<double> &parameters) = 0;
  virtual void writeParameters(std::vector<double> &parameters) const = 0;

  virtual void writeParametersToYamlFile(const std::string &filename) const = 0;

  virtual std::string parametersToString(void) const = 0;

  /// @brief calculates the reprojection distance between points
  /// @param P1
  /// @param P2
  /// @return
  double reprojectionDist(const Eigen::Vector3d &P1,
                          const Eigen::Vector3d &P2) const;

  /// @brief estimate reprojection error
  /// @param object_points series of world coordinate points
  /// @param image_points relationship image points
  /// @param r_vecs rotation series of world to camera frame
  /// @param t_vecs translation series of world to camera frame
  /// @param per_view_errors
  /// @return
  double
  reprojectionError(const std::vector<std::vector<cv::Point3f>> &object_points,
                    const std::vector<std::vector<cv::Point2f>> &image_points,
                    const std::vector<cv::Mat> &r_vecs,
                    const std::vector<cv::Mat> &t_vecs,
                    cv::OutputArray per_view_errors = cv::noArray()) const;

  /// @brief estimate reprojection error
  /// @param P world coordinate point
  /// @param camera_q rotation of world frame to camera frame
  /// @param camera_t translation of world frame to camera frame
  /// @param observed_p
  /// @return
  double reprojectionError(const Eigen::Vector3d &P,
                           const Eigen::Quaterniond &camera_q,
                           const Eigen::Vector3d &camera_t,
                           const Eigen::Vector2d &observed_p) const;

  /// @brief world coordinate points convert to image coordinate [u,v]
  /// @param object_points world coordinate points
  /// @param r_vec rotation vector of world coordinate to camera coordinate
  /// @param t_vec translation of world coordinate to camera coordinate
  /// @param image_points image coordinate [u,v]
  void projectPoints(const std::vector<cv::Point3f> &object_points,
                     const cv::Mat &r_vec, const cv::Mat &t_vec,
                     std::vector<cv::Point2f> &image_points) const;

protected:
  cv::Mat mask_;
};

typedef std::shared_ptr<Camera> CameraPtr;
typedef std::shared_ptr<const Camera> CameraConstPtr;

} // namespace sensor_lab

#endif
