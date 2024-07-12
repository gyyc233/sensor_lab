#include "glog/logging.h"
#include <ceres/ceres.h>
#include <iostream>

// 1. create a way of computing the residual function
struct CostFunctor {
  template <typename T> bool operator()(const T *const x, T *residual) const {
    residual[0] = 10.0 - x[0];
    return true;
  }
};

class QuadraticCostFunction
    : public ceres::SizedCostFunction<1 /* number of residuals */,
                                      1 /* size of first parameter */> {
public:
  bool Evaluate(double const *const *parameters, double *residuals,
                double **jacobians) const override {
    double x = parameters[0][0];

    // f(x) = 10 - x.
    residuals[0] = 10 - x;

    if (jacobians != nullptr && jacobians[0] != nullptr) {
      jacobians[0][0] = -1;
    }

    return true;
  }
};

void numericDerivatives() {
  std::cout << "=============== numeric derivatives =============="
            << std::endl;
  double x = 0.5;
  const double initial_x = x;

  ceres::Problem problem;

  // Set up the only cost function (also known as residual). This uses
  // numeric differentiation to obtain the derivative (jacobian).
  ceres::CostFunction *cost_function =
      new ceres::NumericDiffCostFunction<CostFunctor, ceres::CENTRAL, 1, 1>(
          new CostFunctor);
  problem.AddResidualBlock(cost_function, nullptr, &x);

  // Run the solver!
  ceres::Solver::Options options;
  options.minimizer_progress_to_stdout = true;
  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);

  std::cout << summary.BriefReport() << "\n";
  std::cout << "x : " << initial_x << " -> " << x << "\n";
  std::cout << "=============== numeric derivatives ==============\n\n\n"
            << std::endl;
}

void analyticDerivatives() {
  std::cout << "=============== analytic derivatives =============="
            << std::endl;
  double x = 0.5;
  const double initial_x = x;

  // Build the problem.
  ceres::Problem problem;

  // Set up the only cost function (also known as residual).
  ceres::CostFunction *cost_function = new QuadraticCostFunction;
  problem.AddResidualBlock(cost_function, nullptr, &x);

  // Run the solver!
  ceres::Solver::Options options;
  options.minimizer_progress_to_stdout = true;
  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);

  std::cout << summary.BriefReport() << "\n";
  std::cout << "x : " << initial_x << " -> " << x << "\n";
  std::cout << "=============== analytic derivatives ==============\n\n\n"
            << std::endl;
}

int main(int argc, char **argv) {
  google::InitGoogleLogging(argv[0]);
  std::cout << "=============== auto derivatives ==============" << std::endl;
  // The variable to solve for with its initial value. 用来求解的初始值
  double initial_val = 0.5;
  double x = initial_val;

  // 2. 使用这个cost function 构造最小二乘问题
  ceres::Problem problem;

  // 2.1 设置损失函数（也叫残差），使用自动微分获得导数(derivative (or jacobian)
  // )
  ceres::CostFunction *cost_function =
      new ceres::AutoDiffCostFunction<CostFunctor, 1, 1>(new CostFunctor);
  problem.AddResidualBlock(cost_function, nullptr, &x);

  // 3. run solver
  ceres::Solver::Options optional;
  optional.minimizer_progress_to_stdout = true;
  ceres::Solver::Summary summary;
  ceres::Solve(optional, &problem, &summary);

  // 3.1 描述 solver 状态
  std::cout << summary.BriefReport() << "\n";
  std::cout << "x : " << initial_val << " -> " << x << "\n";
  std::cout << "=============== auto derivatives ==============\n\n\n"
            << std::endl;

  // numeric derivatives
  numericDerivatives();
  analyticDerivatives();

  return 0;
}
