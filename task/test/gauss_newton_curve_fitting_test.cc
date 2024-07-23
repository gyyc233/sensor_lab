#include <Eigen/Core>
#include <Eigen/Dense>
#include <chrono>
#include <iostream>
#include <opencv2/opencv.hpp>

// y = exp(a * x * x + b * x + c)

int main(int argc, char **argv) {
  double ar = 1.0, br = 2.0, cr = 1.0;     // 真实参数值
  double ae = 20.0, be = -10.0, ce = 50.0; // 估计参数值
  int N = 100;                             // 数据点
  double w_sigma = 1.0;                    // 噪声Sigma值
  cv::RNG rng;                             // OpenCV随机数产生器

  std::vector<double> x_data, y_data; // 数据
  for (int i = 0; i < N; i++) {
    double x = i / 100.0;
    x_data.push_back(x);
    y_data.push_back(exp(ar * x * x + br * x + cr) +
                     rng.gaussian(w_sigma * w_sigma));
  }

  int iterations = 100;
  double cost = 0, lastCost = 0;
  std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();

  for (int iter = 0; iter < iterations; iter++) {
    Eigen::Matrix3d H = Eigen::Matrix3d::Zero(); // Hessian = J^T * J
    Eigen::Vector3d b = Eigen::Vector3d::Zero(); // bias = -1 * J^T * f(x)
    cost = 0;                                    // reset cost

    for (int i = 0; i < N; i++) {
      double xi = x_data[i], yi = y_data[i];
      double error =
          yi - exp(ae * xi * xi + be * xi + ce); // 这里使用待优化变量作运算

      // F(X) = yi - exp(ar * xi * xi + br * xi+ cr)
      // 两边取对数，再分别对a b c求偏导
      Eigen::Vector3d jacobian_transpose; // J=[de/da,de/db,de/dc]
                                          // 在这里把它写成列,也就是J^T^
      jacobian_transpose[0] = -xi * xi * exp(ae * xi * xi + be * xi + ce);
      jacobian_transpose[1] = -xi * exp(ae * xi * xi + be * xi + ce);
      jacobian_transpose[2] = -exp(ae * xi * xi + be * xi + ce);

      Eigen::Transpose<Eigen::Vector3d> jacobian =
          jacobian_transpose.transpose();

      H += jacobian_transpose * jacobian;
      b += -1 * jacobian_transpose * error;
      cost += error * error;
    }

    // 求解线性方程 Hx=b
    Eigen::Vector3d dx = H.ldlt().solve(b);
    if (isnan(dx[0])) {
      std::cout << "result is nan! Check initialization values" << std::endl;
      break;
    }

    if (iter > 0 && cost >= lastCost) {
      std::cout << "cost: " << cost << ">= last cost: " << lastCost
                << ", iterator: " << iter << ", break." << std::endl;
      break;
    }

    // 添加 delta
    ae += dx[0];
    be += dx[1];
    ce += dx[2];

    // update last cost
    lastCost = cost;

    std::cout << "total cost: " << cost
              << ", \t\tupdate delta x: " << dx.transpose()
              << "\t\testimated params: " << ae << "," << be << "," << ce
              << std::endl;
  }

  std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
  std::chrono::duration<double> time_used =
      std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1);
  std::cout << "solve time cost = " << time_used.count() << " seconds. "
            << std::endl;

  std::cout << "estimated abc = " << ae << ", " << be << ", " << ce
            << std::endl;

  return 0;
}
