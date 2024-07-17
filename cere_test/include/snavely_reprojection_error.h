// Ceres Solver - A fast non-linear least squares minimizer
// Copyright 2023 Google Inc. All rights reserved.
// http://ceres-solver.org/
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// * Redistributions of source code must retain the above copyright notice,
//   this list of conditions and the following disclaimer.
// * Redistributions in binary form must reproduce the above copyright notice,
//   this list of conditions and the following disclaimer in the documentation
//   and/or other materials provided with the distribution.
// * Neither the name of Google Inc. nor the names of its contributors may be
//   used to endorse or promote products derived from this software without
//   specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//
// Author: sameeragarwal@google.com (Sameer Agarwal)
//
// Templated struct implementing the camera model and residual
// computation for bundle adjustment used by Noah Snavely's Bundler
// SfM system. This is also the camera model/residual for the bundle
// adjustment problems in the BAL dataset. It is templated so that we
// can use Ceres's automatic differentiation to compute analytic
// jacobians.
//
// For details see: http://phototour.cs.washington.edu/bundler/
// and http://grail.cs.washington.edu/projects/bal/

#ifndef MY_CERES_SNAVELY_REPROJECTION_ERROR_H_
#define MY_CERES_SNAVELY_REPROJECTION_ERROR_H_

#include "ceres/rotation.h"

namespace MyCeres {
// namespace my_ceres{
// 计算重投影误差
// 构建针孔相机模型。使用9个参数对摄像机进行参数设置：3个用于旋转，3个用于平移，
// 1个用于焦距和2个用于径向畸变（假定主点位于图像中心）。
struct SnavelyReprojectionError {
  SnavelyReprojectionError(double observed_x, double observed_y)
      : observed_x(observed_x), observed_y(observed_y) {}

  template <typename T>
  bool operator()(const T *const camera, const T *const point,
                  T *residuals) const {
    // 1. 点坐标从世界坐标系转相机坐标系
    // camera[0,1,2] are the angle-axis rotation.
    T p_camera[3]; // point 在相机坐标系下的坐标
    // 对 point 左乘 camera[0,1,2] 进行旋转
    ceres::AngleAxisRotatePoint(camera, point, p_camera);
    // camera[3,4,5] are the translation.
    p_camera[0] += camera[3];
    p_camera[1] += camera[4];
    p_camera[2] += camera[5];

    // 2. 相机坐标系转图像坐标系, 这里取负
    T x_image = -p_camera[0] / p_camera[2];
    T y_image = -p_camera[1] / p_camera[2];

    // 3. 图像去畸变
    const T &k1 = camera[7]; // 相机径向畸变
    const T &k2 = camera[8];
    T r_square = x_image * x_image + y_image * y_image;
    T distortion = 1.0 + k1 * r_square + k2 * r_square * r_square;

    // 4.
    // 图像坐标系去畸变，投影到图像上，注意这里没有+c_x,c_y。数据并没有转到像素坐标系
    const T &focal = camera[6];
    T predicated_x = focal * distortion * x_image;
    T predicated_y = focal * distortion * y_image;

    // 5. 计算重投影误差（在图像坐标系中进行）
    residuals[0] = predicated_x - observed_x;
    residuals[1] = predicated_y - observed_y;

    return true;
  }

  static ceres::CostFunction *Create(const double observed_x,
                                     const double observed_y) {
    // ceres::AutoDiffCostFunction <cost function object, 残差项的维度，
    // 后续的参数描述function object中第N个参数的大小> cost function object ==
    // SnavelyReprojectionError 残差项的维度 == T *residuals， [0][1],
    // 所以设置为2 function object 包含 [const T *const camera, const T *const
    // point] 两个参数项，大小分别是9和3 所以设置为
    // <SnavelyReprojectionError,2,9,3>
    return (new ceres::AutoDiffCostFunction<SnavelyReprojectionError, 2, 9, 3>(
        new SnavelyReprojectionError(observed_x, observed_y)));
  }

private:
  double observed_x; // 与3D点对应的已知的图像图标,是观测值
  double observed_y;
};

// Templated pinhole camera model for used with Ceres.  The camera is
// parameterized using 10 parameters. 4 for rotation, 3 for
// translation, 1 for focal length and 2 for radial distortion. The
// principal point is not modeled (i.e. it is assumed be located at
// the image center).
struct SnavelyReprojectionErrorWithQuaternions {
  // (u, v): the position of the observation with respect to the image
  // center point.
  SnavelyReprojectionErrorWithQuaternions(double observed_x, double observed_y)
      : observed_x(observed_x), observed_y(observed_y) {}

  template <typename T>
  bool operator()(const T *const camera, const T *const point,
                  T *residuals) const {
    // camera[0,1,2,3] is are the rotation of the camera as a quaternion.
    //
    // We use QuaternionRotatePoint as it does not assume that the
    // quaternion is normalized, since one of the ways to run the
    // bundle adjuster is to let Ceres optimize all 4 quaternion
    // parameters without using a Quaternion manifold.
    T p[3];
    QuaternionRotatePoint(camera, point, p);

    p[0] += camera[4];
    p[1] += camera[5];
    p[2] += camera[6];

    // Compute the center of distortion. The sign change comes from
    // the camera model that Noah Snavely's Bundler assumes, whereby
    // the camera coordinate system has a negative z axis.
    const T xp = -p[0] / p[2];
    const T yp = -p[1] / p[2];

    // Apply second and fourth order radial distortion.
    const T &l1 = camera[8];
    const T &l2 = camera[9];

    const T r2 = xp * xp + yp * yp;
    const T distortion = 1.0 + r2 * (l1 + l2 * r2);

    // Compute final projected point position.
    const T &focal = camera[7];
    const T predicted_x = focal * distortion * xp;
    const T predicted_y = focal * distortion * yp;

    // The error is the difference between the predicted and observed position.
    residuals[0] = predicted_x - observed_x;
    residuals[1] = predicted_y - observed_y;

    return true;
  }

  // Factory to hide the construction of the CostFunction object from
  // the client code.
  static ceres::CostFunction *Create(const double observed_x,
                                     const double observed_y) {
    return (new ceres::AutoDiffCostFunction<
            SnavelyReprojectionErrorWithQuaternions, 2, 10, 3>(
        new SnavelyReprojectionErrorWithQuaternions(observed_x, observed_y)));
  }

  double observed_x;
  double observed_y;
};

} // namespace MyCeres

#endif
