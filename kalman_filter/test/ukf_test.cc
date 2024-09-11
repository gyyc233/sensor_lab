#include <iostream>
#include <stdint.h>

#include "types.hpp"
#include "unscented_kalman_filter.hpp"
#include <vector>

static constexpr size_t DIM_X{4};
static constexpr size_t DIM_V{4};
static constexpr size_t DIM_Z{2};
static constexpr size_t DIM_N{2};

kf::Vector<DIM_Z> covertCartesian2Polar(const kf::Vector<DIM_X> &cartesian);

void runExample1();

void runExample2();

kf::Vector<DIM_X> funcF(const kf::Vector<DIM_X> &x,
                        const kf::Vector<DIM_V> &v) {
  kf::Vector<DIM_X> y;
  y[0] = x[0] + x[2] + v[0];
  y[1] = x[1] + x[3] + v[1];
  y[2] = x[2] + v[2];
  y[3] = x[3] + v[3];
  return y;
}

kf::Vector<DIM_Z> funcH(const kf::Vector<DIM_X> &x,
                        const kf::Vector<DIM_N> &n) {
  kf::Vector<DIM_Z> y;

  kf::float32_t px{x[0] + n[0]};
  kf::float32_t py{x[1] + n[1]};

  y[0] = std::sqrt((px * px) + (py * py));
  y[1] = std::atan(py / (px + std::numeric_limits<kf::float32_t>::epsilon()));
  return y;
}

int main() {
  // example 1
  // ukf in Single Iteration
  // runExample1();

  runExample2();

  return 0;
}

void runExample1() {
  std::cout << " Start of Example 1: ===========================" << std::endl;

  kf::Vector<DIM_X> x;
  x << 2.0F, 1.0F, 0.0F, 0.0F;
  std::cout << "x:\n" << x << std::endl;

  kf::Matrix<DIM_X, DIM_X> P;
  P << 0.01F, 0.0F, 0.0F, 0.0F, 0.0F, 0.01F, 0.0F, 0.0F, 0.0F, 0.0F, 0.05F,
      0.0F, 0.0F, 0.0F, 0.0F, 0.05F;
  std::cout << "P:\n" << P << std::endl;

  kf::Matrix<DIM_V, DIM_V> Q;
  Q << 0.05F, 0.0F, 0.0F, 0.0F, 0.0F, 0.05F, 0.0F, 0.0F, 0.0F, 0.0F, 0.1F, 0.0F,
      0.0F, 0.0F, 0.0F, 0.1F;
  std::cout << "Q:\n" << Q << std::endl;

  kf::Matrix<DIM_N, DIM_N> R;
  R << 0.01F, 0.0F, 0.0F, 0.01F;
  std::cout << "R:\n" << R << std::endl;

  kf::Vector<DIM_Z> z;
  z << 2.5F, 0.05F;
  std::cout << "z:\n" << z << std::endl;

  // DIM_V:过程噪声的维度；DIM_N:测量噪声的维度
  kf::UnscentedKalmanFilter<DIM_X, DIM_Z, DIM_V, DIM_N> ukf;

  ukf.vecX() = x;
  ukf.matP() = P;

  ukf.setCovarianceQ(Q);
  ukf.setCovarianceR(R);

  ukf.predictUKF(funcF);

  // Expectation from the python results:
  // =====================================
  // x =
  //     [2.0 1.0 0.0 0.0]
  // P =
  //     [[0.11  0.00  0.05  0.00]
  //      [0.00  0.11  0.00  0.05]
  //      [0.05  0.00  0.15  0.00]
  //      [0.00  0.05  0.00  0.15]]

  // here using funcH but funcF nonlinear function propagate
  ukf.correctUKF(funcH, z);

  // Expectations from the python results:
  // ======================================
  // x =
  //     [ 2.554  0.356  0.252 -0.293]
  // P =
  //     [[ 0.01  -0.001  0.005 -0.    ]
  //      [-0.001  0.01 - 0.     0.005 ]
  //      [ 0.005 - 0.     0.129 - 0.  ]
  //      [-0.     0.005 - 0.     0.129]]

  std::cout << " End of Example 1: ===========================" << std::endl;
}

kf::Vector<DIM_Z> covertCartesian2Polar(const kf::Vector<DIM_X> &cartesian) {
  const kf::Vector<DIM_Z> polar{
      std::sqrt(cartesian[0] * cartesian[0] + cartesian[1] * cartesian[1]),
      std::atan2(cartesian[1], cartesian[0])};
  return polar;
}

void runExample2() {
  std::cout << " Start of Example 2: ===========================" << std::endl;
  kf::Vector<DIM_X> x0, x1, x2, x3, x4, x5, x6;
  x0 << 10.0F, 5.0F, 0.0F, 0.0F;
  x1 << 11.0F, 5.0F, 0.0F, 0.0F;
  x2 << 12.0F, 5.0F, 0.0F, 0.0F;
  x3 << 13.0F, 6.0F, 0.0F, 0.0F;
  x4 << 14.0F, 6.5F, 0.0F, 0.0F;
  x5 << 15.0F, 7.0F, 0.0F, 0.0F;
  x6 << 16.0F, 7.0F, 0.0F, 0.0F;
  std::vector<kf::Vector<DIM_X>> input_x{x0, x1, x2, x3, x4, x5, x6};

  kf::Matrix<DIM_X, DIM_X> P;
  P << 0.1F, 0.0F, 0.0F, 0.0F, 0.0F, 0.1F, 0.0F, 0.0F, 0.0F, 0.0F, 0.1F, 0.0F,
      0.0F, 0.0F, 0.0F, 0.1F;
  std::cout << "P:\n" << P << std::endl;

  kf::Matrix<DIM_V, DIM_V> Q;
  Q << 0.05F, 0.0F, 0.0F, 0.0F, 0.0F, 0.05F, 0.0F, 0.0F, 0.0F, 0.0F, 0.1F, 0.0F,
      0.0F, 0.0F, 0.0F, 0.1F;
  std::cout << "Q:\n" << Q << std::endl;

  kf::Matrix<DIM_N, DIM_N> R;
  R << 0.1F, 0.0F, 0.0F, 0.1F;
  std::cout << "R:\n" << R << std::endl;

  kf::Vector<DIM_Z> vecZ0{covertCartesian2Polar(x0)};
  std::cout << "vecZ0:\n" << vecZ0 << std::endl;
  kf::Vector<DIM_Z> vecZ1{covertCartesian2Polar(x1)};
  std::cout << "vecZ1:\n" << vecZ1 << std::endl;
  kf::Vector<DIM_Z> vecZ2{covertCartesian2Polar(x2)};
  std::cout << "vecZ2:\n" << vecZ2 << std::endl;
  kf::Vector<DIM_Z> vecZ3{covertCartesian2Polar(x3)};
  std::cout << "vecZ3:\n" << vecZ3 << std::endl;
  kf::Vector<DIM_Z> vecZ4{covertCartesian2Polar(x4)};
  std::cout << "vecZ4:\n" << vecZ4 << std::endl;
  kf::Vector<DIM_Z> vecZ5{covertCartesian2Polar(x5)};
  std::cout << "vecZ5:\n" << vecZ5 << std::endl;
  kf::Vector<DIM_Z> vecZ6{covertCartesian2Polar(x6)};
  std::cout << "vecZ6:\n" << vecZ6 << std::endl;
  std::vector<kf::Vector<DIM_Z>> input_z{vecZ0, vecZ1, vecZ2, vecZ3,
                                         vecZ4, vecZ5, vecZ6};

  kf::UnscentedKalmanFilter<DIM_X, DIM_Z, DIM_V, DIM_N> ukf;
  for (size_t i = 0; i < input_x.size(); i++) {
    ukf.vecX() = input_x[i];
    ukf.matP() = P;

    ukf.setCovarianceQ(Q);
    ukf.setCovarianceR(R);

    ukf.predictUKF(funcF);
    ukf.correctUKF(funcH, input_z[i]);
    std::cin.get();
  }
  std::cout << " End of Example 2: ===========================" << std::endl;
}
