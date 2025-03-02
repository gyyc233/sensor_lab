#ifndef G2O_PREINTEGRATION_TYPES_H
#define G2O_PREINTEGRATION_TYPES_H

#include <g2o/core/base_binary_edge.h>
#include <g2o/core/base_multi_edge.h>
#include <g2o/core/base_unary_edge.h>
#include <g2o/core/base_vertex.h>
#include <g2o/core/robust_kernel.h>

#include "eigen_type/eigen_types.h"
#include "imu_preintegration.h"

namespace sad {

/// 与预积分相关的vertex, edge
/**
 * 预积分边
 * 连接6个顶点：上一帧的pose, v, bg, ba，下一帧的pose, v
 * 观测量为9维，即预积分残差, 顺序：R, v, p
 * information从预积分类中获取，构造函数中计算
 */
class EdgeInertial : public g2o::BaseMultiEdge<9, Vec9d> {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /**
   * 构造函数中需要指定预积分类对象
   * @param preinteg  预积分对象指针
   * @param gravity   重力矢量
   * @param weight    权重
   */
  EdgeInertial(std::shared_ptr<IMUPreintegration> preinteg,
               const Vec3d &gravity, double weight = 1.0);

  bool read(std::istream &is) override { return false; }
  bool write(std::ostream &os) const override { return false; }

  /// @brief 使用当前顶点，计算测量值与观测值的误差
  void computeError() override;

  /// @brief 计算待优化变量的偏导 jacobian
  /// @note
  /// 若此函数不被定义，则采用G2O内置的数值求导，但是数值求导很慢，不如解析求导迅速
  void linearizeOplus() override;

  Eigen::Matrix<double, 24, 24> GetHessian() {
    linearizeOplus();
    Eigen::Matrix<double, 9, 24> J;
    J.block<9, 6>(0, 0) = _jacobianOplus[0];
    J.block<9, 3>(0, 6) = _jacobianOplus[1];
    J.block<9, 3>(0, 9) = _jacobianOplus[2];
    J.block<9, 3>(0, 12) = _jacobianOplus[3];
    J.block<9, 6>(0, 15) = _jacobianOplus[4];
    J.block<9, 3>(0, 21) = _jacobianOplus[5];
    return J.transpose() * information() * J;
  }

private:
  const double dt_;
  std::shared_ptr<IMUPreintegration> preint_ = nullptr;
  Vec3d grav_;
};
} // namespace sad

#endif
