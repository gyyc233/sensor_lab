#include <cmath>
#include <cstdio>
#include <iostream>

#include "ceres/ceres.h"
#include "ceres/rotation.h"
#include "glog/logging.h"

class BALProblem {
public:
  ~BALProblem() {
    delete[] point_index_;
    delete[] camera_index_;
    delete[] observations_;
    delete[] parameters_;
  }

  int num_observation() const { return num_observations_; }

  const double *observations() const { return observations_; }

  double *mutable_cameras() { return parameters_; }

  double *mutable_points() { return parameters_ + 9 * num_cameras_; }

  double *mutable_camera_for_observation(int i) {
    return mutable_cameras() + camera_index_[i] * 9;
  }
  double *mutable_point_for_observation(int i) {
    return mutable_points() + point_index_[i] * 3;
  }

  bool LoadFile(const char *filename) {
    FILE *fptr = fopen(filename, "r");
    if (fptr == nullptr) {
      return false;
    };

    FscanfOrDie(fptr, "%d", &num_cameras_);
    FscanfOrDie(fptr, "%d", &num_points_);
    FscanfOrDie(fptr, "%d", &num_observations_);

    printf("num_cameras: %d \n", num_cameras_);
    printf("num_points_: %d \n", num_points_);
    printf("num_observations_: %d\n", num_observations_);

    point_index_ = new int[num_observations_];
    camera_index_ = new int[num_observations_];
    observations_ = new double[2 * num_observations_];

    num_parameters_ = 9 * num_cameras_ + 3 * num_points_;
    parameters_ = new double[num_parameters_];

    for (int i = 0; i < num_observations_; ++i) {
      FscanfOrDie(fptr, "%d", camera_index_ + i);
      FscanfOrDie(fptr, "%d", point_index_ + i);
      for (int j = 0; j < 2; ++j) {
        FscanfOrDie(fptr, "%lf", observations_ + 2 * i + j);
      }
    }

    for (int i = 0; i < num_parameters_; ++i) {
      FscanfOrDie(fptr, "%lf", parameters_ + i);
    }
    return true;
  }

private:
  template <typename T>
  void FscanfOrDie(FILE *fptr, const char *format, T *value) {
    int num_scanned = fscanf(fptr, format, value);
    if (num_scanned != 1) {
      LOG(FATAL) << "invalid UW data file";
    }
  }

  int num_cameras_;
  int num_points_;
  int num_observations_;
  int num_parameters_;

  int *point_index_;
  int *camera_index_;
  double *observations_;
  double *parameters_;
};

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

int main(int argc, char **argv) {
  google::InitGoogleLogging(argv[0]);
  if (argc != 2) {
    std::cerr << "usage: simple_bundle_adjuster <bal_problem>\n";
    return 1;
  }

  BALProblem bal_problem;
  if (!bal_problem.LoadFile(argv[1])) {
    std::cerr << "ERROR: unable to open file " << argv[1] << "\n";
    return 1;
  }

  const double *observations = bal_problem.observations();

  ceres::Problem problem;
  for (int i = 0; i < bal_problem.num_observation(); i++) {
    ceres::CostFunction *cost_function = SnavelyReprojectionError::Create(
        observations[2 * i + 0], observations[2 * i + 1]);
    problem.AddResidualBlock(cost_function, nullptr,
                             bal_problem.mutable_camera_for_observation(i),
                             bal_problem.mutable_point_for_observation(i));
  }

  ceres::Solver::Options options;
  options.linear_solver_type =
      ceres::DENSE_SCHUR; // 对于BA问题，SCHUR 求解器速度会快一点
  options.minimizer_progress_to_stdout = true;

  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);
  std::cout << summary.FullReport() << "\n";
  return 0;
}
