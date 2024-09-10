///
/// Copyright 2022 Mohanad Youssef (Al-khwarizmi)
///
/// Use of this source code is governed by an GPL-3.0 - style
/// license that can be found in the LICENSE file or at
/// https://opensource.org/licenses/GPL-3.0
///
/// @author Mohanad Youssef <mohanad.magdy.hammad@gmail.com>
/// @file main.cpp
///

#include <iostream>
#include <stdint.h>

#include "kalman_filter.h"
#include "types.h"

static constexpr size_t DIM_X{2};
static constexpr size_t DIM_Z{2};

static kf::KalmanFilter<DIM_X, DIM_Z> kalman_filter;

kf::Vector<DIM_Z> covertCartesian2Polar(const kf::Vector<DIM_X> &cartesian);
kf::Matrix<DIM_Z, DIM_Z> calculateJacobianMatrix(const kf::Vector<DIM_X> &vecX);
void executeCorrectionStep();

int main() {
  executeCorrectionStep();

  return 0;
}

kf::Vector<DIM_Z> covertCartesian2Polar(const kf::Vector<DIM_X> &cartesian) {
  const kf::Vector<DIM_Z> polar{
      std::sqrt(cartesian[0] * cartesian[0] + cartesian[1] * cartesian[1]),
      std::atan2(cartesian[1], cartesian[0])};
  return polar;
}

kf::Matrix<DIM_Z, DIM_Z>
calculateJacobianMatrix(const kf::Vector<DIM_X> &vecX) {
  const kf::float32_t valX2PlusY2{(vecX[0] * vecX[0]) + (vecX[1] * vecX[1])};
  const kf::float32_t valSqrtX2PlusY2{std::sqrt(valX2PlusY2)};

  kf::Matrix<DIM_Z, DIM_Z> matHj;
  matHj << (vecX[0] / valSqrtX2PlusY2), (vecX[1] / valSqrtX2PlusY2),
      (-vecX[1] / valX2PlusY2), (vecX[0] / valX2PlusY2);

  return matHj;
}

void executeCorrectionStep() {
  kalman_filter.vecX() << 10.0F, 5.0F;
  kalman_filter.matP() << 0.3F, 0.0F, 0.0F, 0.3F;

  const kf::Vector<DIM_X> measPosCart{10.4F, 5.2F};
  const kf::Vector<DIM_Z> vecZ{covertCartesian2Polar(measPosCart)};

  kf::Matrix<DIM_Z, DIM_Z> matR;
  matR << 0.1F, 0.0F, 0.0F, 0.0008F;

  kf::Matrix<DIM_Z, DIM_X> matHj{
      calculateJacobianMatrix(kalman_filter.vecX())}; // jacobian matrix Hj
  std::cout << "origin H jacobian matrix: " << matHj << std::endl;

  kalman_filter.correctEkf(covertCartesian2Polar, vecZ, matR, matHj);

  std::cout << "\ncorrected state vector = \n" << kalman_filter.vecX() << "\n";
  std::cout << "\ncorrected state covariance = \n"
            << kalman_filter.matP() << "\n";

  std::cout << "updated H jacobian matrix: "
            << calculateJacobianMatrix(kalman_filter.vecX()) << std::endl;
}
