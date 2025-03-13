#ifndef LIDAR_2D_G2O_TYPES_H
#define LIDAR_2D_G2O_TYPES_H
#include <g2o/core/base_binary_edge.h>
#include <g2o/core/base_unary_edge.h>
#include <g2o/core/base_vertex.h>

#include <glog/logging.h>
#include <opencv2/core.hpp>

#include "eigen_type/eigen_types.h"
#include "math_utils.h"

namespace sad {
// 构建g2oSE2顶点和边以及似然场边

/// @brief SE2顶点 (x,y,theta)待优化变量为3,它们的数据类型是SE2
class VertexSE2 : public g2o::BaseVertex<3, SE2> {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /// @brief 估计值清零
  void setToOriginImpl() override { _estimate = SE2(); }

  /// @brief 根据增量，更新估计值
  /// @param update
  void oplusImpl(const double *update) override {
    _estimate.translation()[0] += update[0];
    _estimate.translation()[1] += update[1];
    _estimate.so2() = _estimate.so2() * SO2::exp(update[2]);
  }

  bool read(std::istream &is) override { return true; }

  bool write(std::ostream &os) const override { return true; }
};

/// @brief SE2边，链接两个SE2 (观测值维度，类型，连接顶点类型)
class EdgeSE2 : public g2o::BaseBinaryEdge<3, SE2, VertexSE2, VertexSE2> {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  EdgeSE2() {}

  /// @brief 计算测量值和估计值的差
  void computeError() override {
    VertexSE2 *v1 = (VertexSE2 *)_vertices[0];
    VertexSE2 *v2 = (VertexSE2 *)_vertices[1];
    // error = v1.inv * v2 * meas.inv
    _error =
        (v1->estimate().inverse() * v2->estimate() * measurement().inverse())
            .log();
  }

  // TODO jacobian
  // 这里不重写linearizeOplus 是直接使用G2O内置的数值求导，速度会慢于解析求导

  bool read(std::istream &is) override { return true; }
  bool write(std::ostream &os) const override { return true; }

private:
};

/// @brief 2D似然场边,这里用的是单元边，更新后指向自己
class EdgeSE2LikelihoodFiled : public g2o::BaseUnaryEdge<1, double, VertexSE2> {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

  EdgeSE2LikelihoodFiled(const cv::Mat &field_image, double range, double angle,
                         float resolution = 10.0)
      : field_image_(field_image), range_(range), angle_(angle),
        resolution_(resolution) {}

  /// 判定此条边是否在field image外面
  bool IsOutSide() {
    VertexSE2 *v = (VertexSE2 *)_vertices[0];
    SE2 pose = v->estimate();
    Vec2d pw =
        pose * Vec2d(range_ * std::cos(angle_), range_ * std::sin(angle_));
    Vec2i pf =
        (pw * resolution_ + Vec2d(field_image_.rows / 2, field_image_.cols / 2))
            .cast<int>(); // 图像坐标

    if (pf[0] >= image_boarder_ && pf[0] < field_image_.cols - image_boarder_ &&
        pf[1] >= image_boarder_ && pf[1] < field_image_.rows - image_boarder_) {
      return false;
    } else {
      return true;
    }
  }

  /// @brief 计算估计值和观测值的差
  void computeError() override {
    VertexSE2 *v = (VertexSE2 *)_vertices[0];
    SE2 pose = v->estimate();
    Vec2d pw =
        pose * Vec2d(range_ * std::cos(angle_), range_ * std::sin(angle_));
    Vec2d pf = pw * resolution_ +
               Vec2d(field_image_.rows / 2, field_image_.cols / 2) -
               Vec2d(0.5, 0.5); // 图像坐标

    if (pf[0] >= image_boarder_ && pf[0] < field_image_.cols - image_boarder_ &&
        pf[1] >= image_boarder_ && pf[1] < field_image_.rows - image_boarder_) {
      // 直接使用该点似然场函数的值作为误差(以前做差看多了，到这不太习惯)
      _error[0] = math::GetPixelValue<float>(field_image_, pf[0], pf[1]);
    } else {
      _error[0] = 0;
      setLevel(1);
    }
  }

  /// @brief 计算jacobian
  void linearizeOplus() override {
    VertexSE2 *v = (VertexSE2 *)_vertices[0];
    SE2 pose = v->estimate();
    float theta = pose.so2().log();
    Vec2d pw =
        pose * Vec2d(range_ * std::cos(angle_), range_ * std::sin(angle_));
    Vec2d pf = pw * resolution_ +
               Vec2d(field_image_.rows / 2, field_image_.cols / 2) -
               Vec2d(0.5, 0.5); // 图像坐标

    if (pf[0] >= image_boarder_ && pf[0] < field_image_.cols - image_boarder_ &&
        pf[1] >= image_boarder_ && pf[1] < field_image_.rows - image_boarder_) {
      // 图像梯度
      float dx =
          0.5 * (math::GetPixelValue<float>(field_image_, pf[0] + 1, pf[1]) -
                 math::GetPixelValue<float>(field_image_, pf[0] - 1, pf[1]));
      float dy =
          0.5 * (math::GetPixelValue<float>(field_image_, pf[0], pf[1] + 1) -
                 math::GetPixelValue<float>(field_image_, pf[0], pf[1] - 1));

      _jacobianOplusXi << resolution_ * dx, resolution_ * dy,
          -resolution_ * dx * range_ * std::sin(angle_ + theta) +
              resolution_ * dy * range_ * std::cos(angle_ + theta);
    } else {
      _jacobianOplusXi.setZero();
      setLevel(1);
    }
  }

  bool read(std::istream &is) override { return true; }
  bool write(std::ostream &os) const override { return true; }

private:
  const cv::Mat &field_image_;
  double range_ = 0;
  double angle_ = 0;
  float resolution_ = 10.0;
  inline static const int image_boarder_ = 10;
};

} // namespace sad

#endif
