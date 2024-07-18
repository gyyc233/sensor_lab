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
// Class for loading and holding in memory bundle adjustment problems
// from the BAL (Bundle Adjustment in the Large) dataset from the
// University of Washington.
//
// For more details see http://grail.cs.washington.edu/projects/bal/

#ifndef MY_CERES_BAL_PROBLEM_H_
#define MY_CERES_BAL_PROBLEM_H_

#include <string>

namespace MyCeres {
class BALProblem {
public:
  // explicit可以抑制内置类型隐式转换,所以在类的构造函数中,最好尽可能多用explicit关键字,防止不必要的隐式转换
  explicit BALProblem(const std::string &filename, bool use_quaternions);
  ~BALProblem();

  /// @brief write result to file
  /// @param filename
  void WriteToFile(const std::string &filename) const;

  /// @brief write result to .ply file
  /// @param filename
  void WriteToPLYFile(const std::string &filename, int r = 255, int g = 255,
                      int b = 255) const;

  // Move the "center" of the reconstruction to the origin, where the
  // center is determined by computing the marginal median of the
  // points. The reconstruction is then scaled so that the median
  // absolute deviation of the points measured from the origin is
  // 100.0.
  //
  // The reprojection error of the problem remains the same.
  void Normalize();

  // Perturb(扰乱) the camera pose and the geometry with random normal
  // numbers with corresponding standard deviations.
  // 用特定标准差的随机正态分布值扰乱相机姿态和几何结构
  void Perturb(const double rotation_sigma, const double translation_sigma,
               const double point_sigma);

  // clang-format off
  // 相机参数个数
  int camera_block_size()      const { return use_quaternions_ ? 10 : 9; }
  // 点参数个数
  int point_block_size()       const { return 3;                         }
  int num_cameras()            const { return num_cameras_;              }
  int num_points()             const { return num_points_;               }
  int num_observations()       const { return num_observations_;         }

  // 参数总数(包含9 dimension camera data and 3 dimension point data)
  int num_parameters()         const { return num_parameters_;           }
  const int* point_index()     const { return point_index_;              }
  const int* camera_index()    const { return camera_index_;             }
  const double* observations() const { return observations_;             }
  const double* parameters()   const { return parameters_;               }
  const double* cameras()      const { return parameters_;               }
  double* mutable_cameras()          { return parameters_;               }
  // clang-format on
  double *mutable_points() {
    // points data is behind camera data, so add addr
    return parameters_ + camera_block_size() * num_cameras_;
  }

private:
  /// @brief 相机坐标系通过轴角旋转到世界坐标系
  /// @param camera
  /// @param angle_axis
  /// @param center
  void CameraToAngleAxisAndCenter(const double *camera, double *angle_axis,
                                  double *center) const;

  /// @brief 世界坐标系通过轴角旋转到相机坐标系
  /// @param angle_axis
  /// @param center
  /// @param camera
  void AngleAxisAndCenterToCamera(const double *angle_axis,
                                  const double *center, double *camera) const;
  int num_cameras_;
  int num_points_;
  int num_observations_; // 观测点数量
  int num_parameters_; // size of 9 dimension camera data and 3 dimension point
                       // data
  bool use_quaternions_;

  int *point_index_;
  int *camera_index_;
  double *observations_;
  // The parameter vector is laid out as follows
  // [camera_1, ..., camera_n, point_1, ..., point_m]
  double *parameters_;
};
} // namespace MyCeres

#endif
