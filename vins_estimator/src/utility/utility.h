#ifndef VINS_ESTIMATOR_UTILITY_H
#define VINS_ESTIMATOR_UTILITY_H

#include <cassert>
#include <cmath>
#include <cstring>
#include <eigen3/Eigen/Dense>
#include <libgen.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/stat.h>
#include <sys/types.h>

namespace sensor_lab {

class Utility {
public:
  /// @brief 小角度旋转转对应增量四元数
  /// @note
  /// 近似计算小角度旋转对应的四元数，通常适用于相邻IMU帧间的相对旋转，避免三角函数运算，仅需加减乘除，适合实时系统
  /// @tparam Derived min rotation vector
  /// @param theta
  /// @return 单位四元数，表示小角度旋转
  template <typename Derived>
  static Eigen::Quaternion<typename Derived::Scalar>
  deltaQ(const Eigen::MatrixBase<Derived> &theta) {
    typedef typename Derived::Scalar Scalar_t;

    Eigen::Quaternion<Scalar_t> dq;
    Eigen::Matrix<Scalar_t, 3, 1> half_theta = theta;
    half_theta /= static_cast<Scalar_t>(2.0); // delta /2

    dq.w() = static_cast<Scalar_t>(1.0); // 实部取１
    dq.x() = half_theta.x();             // 虚部取 delta /2
    dq.y() = half_theta.y();
    dq.z() = half_theta.z();

    // IMU预积分中计算相邻IMU帧间的相对旋转增量
    // 在构建IMU残差时，计算预测旋转与观测旋转的差异
    return dq;
  }

  /// @brief 计算三维向量的反对称矩阵
  /// @tparam Derived
  /// @param q rotation vector
  /// @return
  template <typename Derived>
  static Eigen::Matrix<typename Derived::Scalar, 3, 3>
  skewSymmetric(const Eigen::MatrixBase<Derived> &q) {
    Eigen::Matrix<typename Derived::Scalar, 3, 3> ans;
    ans << typename Derived::Scalar(0), -q(2), q(1), q(2),
        typename Derived::Scalar(0), -q(0), -q(1), q(0),
        typename Derived::Scalar(0);
    return ans;
  }

  /// @brief 确保输入的四元数（quaternion）的实部（即w分量）为正数
  /// @note
  /// 在四元数表示中q和-q表示的是同一个旋转方向，但为了便于比较或插值运算，通常希望保持一致性，例如都使用w>=
  /// 0的四元数
  /// @tparam Derived
  /// @param q
  /// @return
  template <typename Derived>
  static Eigen::Quaternion<typename Derived::Scalar>
  positify(const Eigen::QuaternionBase<Derived> &q) {
    return q.template w() >= (typename Derived::Scalar)(0.0)
               ? q
               : Eigen::Quaternion<typename Derived::Scalar>(-q.w(), -q.x(),
                                                             -q.y(), -q.z());
  }

  /// @brief 构造单位四元数的左乘矩阵
  /// @note 四元数左乘矩阵是指将四元数的乘法运算转化为矩阵形式,单位四元数乘法
  /// q1*q2 = Qleft(q1)*q2, 可用于旋转更新与构建imu残差时四元数乘法对变量求导
  /// @tparam Derived
  /// @param q
  /// @return
  template <typename Derived>
  static Eigen::Matrix<typename Derived::Scalar, 4, 4>
  Qleft(const Eigen::QuaternionBase<Derived> &q) {
    Eigen::Quaternion<typename Derived::Scalar> qq =
        positify(q); // 确保四元数的实部>=0
    Eigen::Matrix<typename Derived::Scalar, 4, 4> ans;
    ans(0, 0) = qq.w(); // 第一行第一个元素是w
    ans.template block<1, 3>(0, 1) =
        -qq.vec().transpose(); // 后三个元素是负的向量部分转置
    ans.template block<3, 1>(1, 0) =
        qq.vec(); // 第一列下面的三个元素是四元数的向量部分 v

    // 右下角的 3x3 子矩阵是旋转向量的skew矩阵
    ans.template block<3, 3>(1, 1) =
        qq.w() * Eigen::Matrix<typename Derived::Scalar, 3, 3>::Identity() +
        skewSymmetric(qq.vec());
    return ans;
  }

  template <typename Derived>
  static Eigen::Matrix<typename Derived::Scalar, 4, 4>
  Qright(const Eigen::QuaternionBase<Derived> &q) {
    Eigen::Quaternion<typename Derived::Scalar> qq =
        positify(q); // 确保四元数的实部>=0
    Eigen::Matrix<typename Derived::Scalar, 4, 4> ans;
    ans(0, 0) = qq.w(); // 第一行第一个元素是w
    ans.template block<1, 3>(0, 1) =
        -qq.vec().transpose(); // 后三个元素是负的向量部分转置
    ans.template block<3, 1>(1, 0) =
        qq.vec(); // 第一列下面的三个元素是四元数的向量部分 v

    // 右下角的 3x3 子矩阵是旋转向量的skew矩阵
    ans.template block<3, 3>(1, 1) =
        qq.w() * Eigen::Matrix<typename Derived::Scalar, 3, 3>::Identity() +
        (-1.0) * skewSymmetric(qq.vec());
    return ans;
  }

  /// @brief rotation matrix to ypr
  /// @note R=r_z*r_y*r_x
  /// @param R
  /// @return ypr(deg)
  static Eigen::Vector3d R2ypr(const Eigen::Matrix3d &R) {
    Eigen::Vector3d n = R.col(0);
    Eigen::Vector3d o = R.col(1);
    Eigen::Vector3d a = R.col(2);

    Eigen::Vector3d ypr(3);
    double y = atan2(n(1), n(0)); // yaw 物体在水平面上的朝向
    double p = atan2(-n(2), n(0) * cos(y) +
                                n(1) * sin(y)); // pitch 投影到 x-z 平面后的夹角
    double r = atan2(a(0) * sin(y) - a(1) * cos(y),
                     -o(0) * sin(y) + o(1) * cos(y)); // roll
    ypr(0) = y;
    ypr(1) = p;
    ypr(2) = r;

    return ypr / M_PI * 180.0;
  }

  /// @brief  Convert a rotation matrix to a quaternion (Rz * Ry * Rx)
  /// @tparam Derived
  /// @param ypr
  /// @return
  template <typename Derived>
  static Eigen::Matrix<typename Derived::Scalar, 3, 3>
  ypr2R(const Eigen::MatrixBase<Derived> &ypr) {
    typedef typename Derived::Scalar Scalar_t;

    Scalar_t y = ypr(0) / 180.0 * M_PI;
    Scalar_t p = ypr(1) / 180.0 * M_PI;
    Scalar_t r = ypr(2) / 180.0 * M_PI;

    Eigen::Matrix<Scalar_t, 3, 3> Rz;
    Rz << cos(y), -sin(y), 0, sin(y), cos(y), 0, 0, 0, 1;

    Eigen::Matrix<Scalar_t, 3, 3> Ry;
    Ry << cos(p), 0., sin(p), 0., 1., 0., -sin(p), 0., cos(p);

    Eigen::Matrix<Scalar_t, 3, 3> Rx;
    Rx << 1., 0., 0., 0., cos(r), -sin(r), 0., sin(r), cos(r);

    return Rz * Ry * Rx;
  }

  /// @brief 根据输入的重力向量 g 计算出一个对应的旋转矩阵
  /// R0,用于描述从该重力方向到某个目标坐标系（通常是世界坐标系）之间的旋转关系
  /// @param g
  /// @return
  static Eigen::Matrix3d g2R(const Eigen::Vector3d &g);

  // 这是一个模板结构体，用于在编译时将整数作为类型使用。它不包含任何数据成员，仅作为模板元编程中的标记类型（tag
  // type），用于区分不同的模板特化版本
  template <size_t N> struct uint_ {};

  // 基于模板的循环展开函数实现，通过递归调用自身来展开循环
  // 它首先递归调用 unroller(f, iter, uint_<N - 1>())，直到到达 N == 0
  // 的基本情况 然后执行 f(iter + N)，即对第 N 个元素应用函数 f
  // 由于递归是先执行的，因此展开顺序是从低索引到高索引

  // // 假设 iter 是一个指针或迭代器，指向某个数组起始位置
  // unroller(uint_<3>(), [](auto* ptr) { std::cout << *ptr << std::endl; },
  // iter); 这段代码会展开为以下逻辑: f(iter + 0).f(iter + 1).f(iter + 2).f(iter
  // + 3)
  template <size_t N, typename Lambda, typename IterT>
  void unroller(const Lambda &f, const IterT &iter, uint_<N>) {
    unroller(f, iter, uint_<N - 1>());
    f(iter + N);
  }

  /// 当N=0时，递归结束
  template <typename Lambda, typename IterT>
  void unroller(const Lambda &f, const IterT &iter, uint_<0>) {
    f(iter);
  }

  /// @brief Normalize the angle to the range of [-180,180]
  /// @tparam T
  /// @param angle_degrees
  /// @return
  template <typename T> static T normalizeAngle(const T &angle_degrees) {
    T two_pi(2.0 * 180);
    if (angle_degrees > 0)
      return angle_degrees -
             two_pi * std::floor((angle_degrees + T(180)) / two_pi);
    else
      return angle_degrees +
             two_pi * std::floor((-angle_degrees + T(180)) / two_pi);
  };
};

class FileSystemHelper {
public:
  /******************************************************************************
   * Recursively create directory if `path` not exists.
   * Return 0 if success.
   *****************************************************************************/
  static int createDirectoryIfNotExists(const char *path) {
    struct stat info;
    int statRC = stat(path, &info); // 检查路径是否存在
    if (statRC != 0) {
      // 如果路径不存在
      if (errno == ENOENT) {
        printf("%s not exists, trying to create it \n", path);
        // dirname 获取当前路径的父目录路径字符串
        if (!createDirectoryIfNotExists(dirname(strdupa(path)))) {
          if (mkdir(path, S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH) != 0) {
            fprintf(stderr, "Failed to create folder %s \n", path);
            return 1;
          } else
            return 0;
        } else
          return 1;
      } // 如果路径存在但不是目录
      if (errno == ENOTDIR) {
        fprintf(stderr, "%s is not a directory path \n", path);
        return 1;
      } // something in path prefix is not a dir
      return 1;
    }
    return (info.st_mode & S_IFDIR) ? 0 : 1;
  }
};

} // namespace sensor_lab

#endif
