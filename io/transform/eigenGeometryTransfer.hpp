#ifndef __EIGEN_GEOMETRY_TRANSFER_HPP__
#define __EIGEN_GEOMETRY_TRANSFER_HPP__

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <iostream>
#include <math.h>
#include <stdlib.h>
#include <vector>

/// @brief fixed axis rotation euler to quaternion, first around Z axis, then Y
/// axis, last X axis
/// @param roll (x), unit: deg
/// @param pitch (y), unit: deg
/// @param yaw (z), unit: deg
/// @return quaternion
Eigen::Quaterniond euler2QuaternionFixedAxis(const double roll,
                                             const double pitch,
                                             const double yaw);

/// @brief self axis rotation euler to quaternion, first around Z axis, then Y
/// axis, last X axis
/// @param roll (x), unit: deg
/// @param pitch (y), unit: deg
/// @param yaw (z), unit: deg
/// @return quaternion
Eigen::Quaterniond euler2QuaternionSelfAxis(const double roll,
                                            const double pitch,
                                            const double yaw);

void printfQuaternion(const Eigen::Quaterniond &quaternion);

/// @brief euler convert rotation matrix
/// @note
/// 欧拉角定义：一共旋转三次，第一次绕自身Z轴旋转，第二次绕参考坐标系Y轴旋转，第三次绕参考坐标系X轴旋转
/// @param roll (x), unit: deg
/// @param pitch (y), unit: deg
/// @param yaw (z), unit: deg
/// @return rotation matrix
Eigen::Matrix3d euler2RotationMatrixFixedAxis(const double roll,
                                              const double pitch,
                                              const double yaw);

/// @brief euler convert rotation matrix
/// @note
/// 欧拉角定义：一共旋转三次，第一次绕自身Z轴旋转，第二次绕自身Y轴旋转，第三次绕自身X轴旋转
/// @param roll (x), unit: deg
/// @param pitch (y), unit: deg
/// @param yaw (z), unit: deg
/// @return rotation matrix
Eigen::Matrix3d euler2RotationMatrixSelfAxis(const double roll,
                                             const double pitch,
                                             const double yaw);

/// @brief quaternion convert euler, euler: self axis, first Z, last X
/// @param x quaternion x
/// @param y quaternion y
/// @param z quaternion z
/// @param w quaternion w
/// @return euler (rad) [x,y,z]
Eigen::Vector3d quaternion2Euler(const double x, const double y, const double z,
                                 const double w);

Eigen::Matrix3d quaternion2RotationMatrix(const double x, const double y,
                                          const double z, const double w);

Eigen::Quaterniond rotationMatrix2Quaternion(Eigen::Matrix3d R);

/// @brief rotation matrix convert euler, euler: self axis, first Z, last X
/// @param R rotation matrix
/// @return euler (rad) [x,y,z]
Eigen::Vector3d rotationMatrix2euler(Eigen::Matrix3d R);

void printfQuaternion(const Eigen::Quaterniond &quaternion) {
  std::cout << "x = " << quaternion.x() << std::endl;
  std::cout << "y = " << quaternion.y() << std::endl;
  std::cout << "z = " << quaternion.z() << std::endl;
  std::cout << "w = " << quaternion.w() << std::endl << std::endl;
}

Eigen::Quaterniond euler2QuaternionFixedAxis(double roll, double pitch,
                                             double yaw) {
  Eigen::AngleAxisd yaw_angle(yaw / 180.0 * M_PI, Eigen::Vector3d::UnitZ());
  Eigen::AngleAxisd pitch_angle(pitch / 180.0 * M_PI, Eigen::Vector3d::UnitY());
  Eigen::AngleAxisd roll_angle(roll / 180.0 * M_PI, Eigen::Vector3d::UnitX());

  // rotation matrix convert quaternion
  Eigen::Quaterniond q = roll_angle * pitch_angle * yaw_angle;
  std::cout << "eigen euler2Quaternion fixed axis rotation result is:"
            << std::endl;
  printfQuaternion(q);
  return q;
}

Eigen::Quaterniond euler2QuaternionSelfAxis(const double roll,
                                            const double pitch,
                                            const double yaw) {
  Eigen::AngleAxisd yaw_angle(yaw / 180.0 * M_PI, Eigen::Vector3d::UnitZ());
  Eigen::AngleAxisd pitch_angle(pitch / 180.0 * M_PI, Eigen::Vector3d::UnitY());
  Eigen::AngleAxisd roll_angle(roll / 180.0 * M_PI, Eigen::Vector3d::UnitX());

  // rotation matrix convert quaternion
  Eigen::Quaterniond q = yaw_angle * pitch_angle * roll_angle;
  std::cout << "eigen euler2Quaternion self axis rotation result is:"
            << std::endl;
  printfQuaternion(q);

  double cy = cos(yaw / 180.0 * M_PI * 0.5);
  double sy = sin(yaw / 180.0 * M_PI * 0.5);
  double cp = cos(pitch / 180.0 * M_PI * 0.5);
  double sp = sin(pitch / 180.0 * M_PI * 0.5);
  double cr = cos(roll / 180.0 * M_PI * 0.5);
  double sr = sin(roll / 180.0 * M_PI * 0.5);

  double test_w, test_x, test_y, test_z;
  test_w = cy * cp * cr + sy * sp * sr;
  test_x = cy * cp * sr - sy * sp * cr;
  test_y = sy * cp * sr + cy * sp * cr;
  test_z = sy * cp * cr - cy * sp * sr;
  Eigen::Quaterniond test_q(test_w, test_x, test_y, test_z);
  std::cout << "eigen Euler2Quaternion self axis rotation test result is:"
            << std::endl;
  printfQuaternion(test_q);
  return q;
}

Eigen::Matrix3d euler2RotationMatrixFixedAxis(const double roll,
                                              const double pitch,
                                              const double yaw) {
  Eigen::Matrix3d matrix = euler2QuaternionFixedAxis(roll, pitch, yaw).matrix();
  std::cout << "euler2RotationMatrixFixedAxis result is: " << std::endl;
  std::cout << matrix << std::endl;
  return matrix;
}

Eigen::Matrix3d euler2RotationMatrixSelfAxis(const double roll,
                                             const double pitch,
                                             const double yaw) {
  Eigen::Matrix3d matrix = euler2QuaternionSelfAxis(roll, pitch, yaw).matrix();
  std::cout << "euler2QuaternionSelfAxis result is: " << std::endl;
  std::cout << matrix << std::endl;
  return matrix;
}

Eigen::Vector3d quaternion2Euler(const double x, const double y, const double z,
                                 const double w) {

  Eigen::Quaterniond q;
  q.x() = x;
  q.y() = y;
  q.z() = z;
  q.w() = w;

  // Vector3f ea = mat.eulerAngles(2, 0, 2); equivalent with
  // mat == AngleAxisf(ea[0], Vector3f::UnitZ())
  // * AngleAxisf(ea[1], Vector3f::UnitX())
  // * AngleAxisf(ea[2], Vector3f::UnitZ());

  Eigen::Vector3d euler = q.toRotationMatrix().eulerAngles(2, 1, 0);
  std::cout << "quaterniond2Euler result(self rotation, first Z, last X) is:"
            << std::endl;
  std::cout << "x(deg) = " << euler[2] / M_PI * 180.0 << std::endl;
  std::cout << "y(deg) = " << euler[1] / M_PI * 180.0 << std::endl;
  std::cout << "z(deg) = " << euler[0] / M_PI * 180.0 << std::endl << std::endl;

  std::cout << "the same quaternion cna be represented by 2 type of euler "
               "angles, you can try the following method"
            << std::endl;

  double roll, pitch, yaw;
  // roll (x-axis rotation)
  double sinr_cosp = 2.0 * (q.w() * q.x() + q.y() * q.z());
  double cosr_cosp = 1.0 - 2.0 * (q.x() * q.x() + q.y() * q.y());
  roll = atan2(sinr_cosp, cosr_cosp); // atan2: [-pi,pi] atan: [-pi/2, pi/2]

  // pitch (y-axis rotation)
  double sinp = 2.0 * (q.w() * q.y() - q.z() * q.x());
  if (fabs(sinp) >= 1)
    pitch = copysign(M_PI / 2, sinp); // use 90 degrees if out of range
  else
    pitch = asin(sinp);

  // yaw (z-axis rotation)
  double siny_cosp = 2.0 * (q.w() * q.z() + q.x() * q.y());
  double cosy_cosp = 1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z());
  yaw = atan2(siny_cosp, cosy_cosp);

  std::cout << "[roll pitch yaw](deg): " << roll / M_PI * 180.0 << ", "
            << pitch / M_PI * 180.0 << ", " << yaw / M_PI * 180.0 << std::endl;
  return euler;
}

Eigen::Matrix3d quaternion2RotationMatrix(const double x, const double y,
                                          const double z, const double w) {
  Eigen::Quaterniond q;
  q.x() = x;
  q.y() = y;
  q.z() = z;
  q.w() = w;
  Eigen::Matrix3d R = q.normalized().toRotationMatrix();
  std::cout << "quaternion2RotationMatrix result is:" << std::endl;
  std::cout << R << std::endl;
  return R;
}

Eigen::Quaterniond rotationMatrix2Quaternion(Eigen::Matrix3d R) {
  Eigen::Quaterniond q = Eigen::Quaterniond(R);
  q.normalize();
  std::cout << "rotationMatrix2Quaterniond result is:" << std::endl;
  std::cout << "x = " << q.x() << std::endl;
  std::cout << "y = " << q.y() << std::endl;
  std::cout << "z = " << q.z() << std::endl;
  std::cout << "w = " << q.w() << std::endl;
  return q;
}

Eigen::Vector3d rotationMatrix2euler(Eigen::Matrix3d R) {
  Eigen::Matrix3d m;
  m = R;
  Eigen::Vector3d euler = m.eulerAngles(2, 1, 0);
  std::cout << "rotationMatrix2euler result is:" << std::endl;
  std::cout << "x(deg) = " << euler[2] / M_PI * 180.0 << std::endl;
  std::cout << "y(deg) = " << euler[1] / M_PI * 180.0 << std::endl;
  std::cout << "z(deg) = " << euler[0] / M_PI * 180.0 << std::endl;
  return euler;
}

#endif
