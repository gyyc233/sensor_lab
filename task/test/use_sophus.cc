#include <cmath>
#include <iostream>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "sophus/se3.h"
#include "sophus/so3.h"

int main() {
  // 沿Z轴转90度的旋转矩阵
  // 　                        角度　　 轴　　罗德里格公式进行转换为旋转矩阵
  Eigen::Matrix3d R =
      Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d(0, 0, 1)).toRotationMatrix();

  //--------------------------------------------------------------------------------------------------
  //  李群
  //--------------------------------------------------------------------------------------
  Sophus::SO3 SO3_R(R); // Sophus::SO(3)可以直接从旋转矩阵构造
  Sophus::SO3 SO3_V(0, 0, M_PI / 2); // 亦可从旋转向量构造
  Eigen::Quaterniond q(R);           // 或者四元数
  Sophus::SO3 SO3_Q(q);

  std::cout << "rotation matrix:\n" << R << std::endl;
  std::cout << "SO(3) from matrix: " << SO3_R << std::endl;
  std::cout << "SO(3) from vector: " << SO3_V << std::endl;
  std::cout << "SO(3) from quaternion :" << SO3_Q << std::endl;

  //--------------------------------------------------------------------------------------
  //  李代数
  //--------------------------------------------------------------------------------------
  // 使用对数映射获得它的李代数
  Eigen::Vector3d so3 = SO3_R.log();
  std::cout << "so3 = " << so3.transpose() << std::endl;
  // hat 为向量到反对称矩阵
  std::cout << "so3 hat=\n" << Sophus::SO3::hat(so3) << std::endl;
  // 相对的，vee为反对称到向量
  std::cout << "so3 hat vee= "
            << Sophus::SO3::vee(Sophus::SO3::hat(so3)).transpose()
            << std::endl; // transpose纯粹是为了输出美观一些

  //--------------------------------------------------------------------------------------------------
  //  李代数求导 更新
  //--------------------------------------------------------------------------------------
  // 增量扰动模型的更新
  Eigen::Vector3d update_so3(1e-4, 0, 0); //假设更新量为这么多
  Sophus::SO3 SO3_updated = Sophus::SO3::exp(update_so3) * SO3_R;
  std::cout << "SO3 updated = " << SO3_updated << std::endl;

  //--------------------------------------------------------------------------------------------------
  //  特殊欧式群  变换矩阵群
  //--------------------------------------------------------------------------------------
  // 对SE(3)操作大同小异
  std::cout << "***************************************" << std::endl;
  Eigen::Vector3d t(1, 0, 0); // 沿X轴平移1
  Sophus::SE3 SE3_Rt(R, t);   // 从R,t构造SE(3)
  Sophus::SE3 SE3_qt(q, t);   // 从q,t构造SE(3)
  std::cout << "SE3 from R,t= " << std::endl << SE3_Rt << std::endl;
  std::cout << "SE3 from q,t= " << std::endl << SE3_qt << std::endl;
  // 李代数se(3) 是一个六维向量，方便起见先typedef一下
  typedef Eigen::Matrix<double, 6, 1> Vector6d;
  Vector6d se3 = SE3_Rt.log();
  std::cout << "se3 = " << se3.transpose() << std::endl;
  // 观察输出，会发现在Sophus中，se(3)的平移在前，旋转在后.
  // 同样的，有hat和vee两个算符
  std::cout << "se3 hat = " << std::endl << Sophus::SE3::hat(se3) << std::endl;
  std::cout << "se3 hat vee = "
            << Sophus::SE3::vee(Sophus::SE3::hat(se3)).transpose() << std::endl;

  // 最后，演示一下更新
  Vector6d update_se3; //更新量
  update_se3.setZero();
  update_se3(0, 0) = 1e-4d;
  Sophus::SE3 SE3_updated = Sophus::SE3::exp(update_se3) * SE3_Rt;
  std::cout << "SE3 updated = " << std::endl
            << SE3_updated.matrix() << std::endl;

  return 0;
}
