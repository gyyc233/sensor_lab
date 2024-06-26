#include <Eigen/Core>
#include <Eigen/Dense>
#include <ctime>
#include <glog/logging.h>
#include <iostream>

#define MATRIX_SIZE 50

int main() {
  // 1. create and initialization matrix
  Eigen::Matrix<float, 2, 3> matrix_23; // declaration row 2 cols 3 matrix

  Eigen::Matrix<float, 3, 1> vd_3d; // declaration row 3 cols 1 matrix
  Eigen::Vector3d v_3d;

  // zero matrix
  // Matrix3d == Eigen::Matrix<double, 3, 3> row 3, cols 3
  Eigen::Matrix3d matrix_33 = Eigen::Matrix3d::Zero();

  // dynamic matrix declaration
  Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> matrix_dynamic;
  Eigen::MatrixXd matrix_x;

  // set size but utilization
  Eigen::MatrixXf a(10, 15);
  Eigen::VectorXf b(30);

  // 2. initialization
  matrix_23 << 1, 2, 3, 4, 5, 6;
  // get element
  for (int i = 0; i < 2; i++) {
    for (int j = 0; j < 3; j++)
      std::cout << matrix_23(i, j) << "\t";
    std::cout << std::endl;
  }

  // random matrix
  matrix_33 = Eigen::Matrix3d::Random();

  // 3. operator
  v_3d << 3, 2, 1;
  vd_3d << 4, 5, 6;

  // multiplication
  // error (failed to float-->double) : Eigen::Matrix<double, 2, 1>
  // result_wrong_type = matrix_23 * v_3d; explicit conversion
  Eigen::Matrix<double, 2, 1> result = matrix_23.cast<double>() * v_3d;
  std::cout << "result: " << result << std::endl;

  Eigen::Matrix<float, 2, 1> result2 = matrix_23 * vd_3d;
  std::cout << "result2: " << result2 << std::endl;

  std::cout << "matrix_33:\n" << matrix_33 << std::endl;
  std::cout << "matrix_33.transpose():\n"
            << matrix_33.transpose() << std::endl; // transpose
  std::cout << "matrix_33.sum():\n"
            << matrix_33.sum() << std::endl; // sum of elements
  std::cout << "matrix_33.trace():\n" << matrix_33.trace() << std::endl;
  std::cout << "10 * matrix_33:\n" << 10 * matrix_33 << std::endl;
  std::cout << "matrix_33.inverse():\n" << matrix_33.inverse() << std::endl;
  std::cout << "matrix_33.determinant():\n"
            << matrix_33.determinant() << std::endl;

  // 4. eigenvalue solving
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eigen_solver(
      matrix_33.transpose() * matrix_33);
  std::cout << "Eigen values = \n" << eigen_solver.eigenvalues() << std::endl;
  std::cout << "Eigen vectors = \n" << eigen_solver.eigenvectors() << std::endl;

  // 5. equation
  // solve matrix_NN * x = v_Nd equation
  Eigen::Matrix<double, MATRIX_SIZE, MATRIX_SIZE> matrix_NN;
  matrix_NN = Eigen::MatrixXd::Random(MATRIX_SIZE, MATRIX_SIZE);
  Eigen::Matrix<double, MATRIX_SIZE, 1> v_Nd;
  v_Nd = Eigen::MatrixXd::Random(MATRIX_SIZE, 1);

  clock_t time_stt = clock(); // 计时函数
  // (1). calculate inverse
  Eigen::Matrix<double, MATRIX_SIZE, 1> x = matrix_NN.inverse() * v_Nd;
  std::cout << "time use in normal inverse is "
            << 1000 * (clock() - time_stt) / (double)CLOCKS_PER_SEC << "ms"
            << std::endl;

  // (2). QR decomposition
  time_stt = clock();
  x = matrix_NN.colPivHouseholderQr().solve(v_Nd);
  std::cout << "time use in Qr decomposition is "
            << 1000 * (clock() - time_stt) / (double)CLOCKS_PER_SEC << "ms"
            << std::endl;
  return 0;
}
