#include "glog/logging.h"
#include <ceres/ceres.h>
#include <chrono>
#include <iostream>
#include <opencv2/core/core.hpp>

struct CurveFittingResidual {

  CurveFittingResidual(double x, double y) : x_(x), y_(y) {}

  template <typename T>
  bool operator()(const T *const a, const T *const b, const T *const c,
                  T *residuals) const {
    residuals[0] = y_ - exp(a[0] * x_ * x_ + b[0] * x_ + c[0]);
    return true;
  }

private:
  const double x_;
  const double y_; // observer
};

int main(int argc, char **argv) {
  google::InitGoogleLogging(argv[0]);
  double ar = 1.0, br = 2.0, cr = 1.0; // 真实参数值

  const double initial_a = 1; // 估计值的初始值
  const double initial_b = 1;
  const double initial_c = 1;

  // build test data
  int N = 100;          // 数据点
  double w_sigma = 1.0; // 噪声Sigma值
  cv::RNG rng;          // OpenCV随机数产生器

  double a = initial_a, b = initial_b, c = initial_c;
  std::vector<double> x_data, y_data; // 数据
  for (int i = 0; i < N; i++) {
    double x = i / 100.0;
    x_data.push_back(x);
    y_data.push_back(exp(ar * x * x + br * x + cr) +
                     rng.gaussian(w_sigma * w_sigma));
  }

  ceres::Problem problem;
  for (int i = 0; i < N; i++) {
    CurveFittingResidual *curve_fitting_residual =
        new CurveFittingResidual(x_data[i], y_data[i]);

    // 模板参数解释 <cost function object, residual dimension, optimizer
    // value1_dimension, optimizer value2_dimension, optimizer value3_dimension>
    ceres::AutoDiffCostFunction<CurveFittingResidual, 1, 1, 1, 1>
        *auto_diff_cost_func =
            new ceres::AutoDiffCostFunction<CurveFittingResidual, 1, 1, 1, 1>(
                curve_fitting_residual);

    // new ceres::CauchyLoss(0.5)

    problem.AddResidualBlock(auto_diff_cost_func, nullptr, &a, &b, &c);
  }

  ceres::Solver::Options option;
  option.max_num_iterations = 100;
  option.linear_solver_type = ceres::DENSE_QR;
  option.minimizer_progress_to_stdout = true;

  std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
  ceres::Solver::Summary summary;
  ceres::Solve(option, &problem, &summary);
  std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
  std::chrono::duration<double> time_used =
      std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1);
  std::cout << "solve time cost = " << time_used.count() << " seconds. "
            << std::endl;

  std::cout << summary.BriefReport() << "\n";
  std::cout << "Initial a: " << initial_a << " b: " << initial_b
            << ", c: " << initial_c << "\n";
  std::cout << "Final a: " << a << " b: " << b << ", c: " << c << "\n";
  return 0;
}
