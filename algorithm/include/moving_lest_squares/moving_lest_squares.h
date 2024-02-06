//--------------------------------------------------------------------------------------------------
//
//  The Implementation of Moving Least Squares Method
//  Reference: http://www.nealen.com/projects/mls/asapmls.pdf
//             https://weijun-lin.top/2021/01/07/2021-01-07-MLS/
//
//--------------------------------------------------------------------------------------------------

#include "Eigen/Dense"
#include <iostream>
#include <vector>

namespace Algorithm {

class MLS {
public:
  MLS() = default;
  virtual double fit(Eigen::VectorXd x);

protected:
  // default [1, x, x^2]
  virtual Eigen::VectorXd basis(const Eigen::VectorXd xi) {
    Eigen::VectorXd bx(m);
    bx(0) = 1;
    bx(1) = xi(0);
    bx(2) = xi(0) * xi(0);
    return bx;
  }

  // 权函数
  virtual double wf(double d, double h) {
    // Gaussian
    static double h2 = 1.0 / (h * h);
    return std::exp(-d * d * h2);

    // Wendland
    // static double inv_h = 1.0 / h;
    // return std::pow(1 - d * inv_h, 4) * (4.0 * d * inv_h + 1);

    // return 1.0 / (d * d + 0.0001);
  }

public:
  std::vector<Eigen::VectorXd> X;
  Eigen::VectorXd Y;

public:
  // h is a spacing parameter which can be used to smooth out
  // small features in the data, see[Levin 2003; Alexa et al. 2003].
  double h = 0.1;

  // d spatial dimensions
  int d = 1;

  // The number of terms in a polynomial
  // In paper <<An As-Short-As-Possible Introduction to the Least Squares,
  // Weighted Least Squaresand Moving Least Squares Methods for Scattered Data
  // Approximationand Interpolation>> m represents total degree
  int m = 3;
};

//--------------------------------------------------------------------------------------------------
//
// Wrapper of MLS
//
//--------------------------------------------------------------------------------------------------
class MLSCurvedSurface : public MLS {
public:
  MLSCurvedSurface() : MLS() {
    d = 2;
    m = 6;
  }

protected:
  Eigen::VectorXd basis(const Eigen::VectorXd xi) override {
    Eigen::VectorXd bx(m);
    bx(0) = 1;
    bx(1) = xi(0);
    bx(2) = xi(1);
    bx(3) = xi(0) * xi(0);
    bx(4) = xi(0) * xi(1);
    bx(5) = xi(1) * xi(1);
    return bx;
  }
};

class MLSCurvedLine : public MLS {
public:
  void SetX(std::vector<double> data_x) {
    X = std::vector<Eigen::VectorXd>(data_x.size(), Eigen::VectorXd{});
    for (int i = 0; i < data_x.size(); i++) {
      Eigen::VectorXd x(1);
      x(0) = data_x[i];
      X[i] = x;
      std::cout << i << " x points:\n" << X[i] << std::endl;
    }
  }

  void SetY(std::vector<double> data_y) {
    Y = Eigen::VectorXd(data_y.size());
    for (int i = 0; i < data_y.size(); i++) {
      Y(i) = data_y[i];
    }

    std::cout << "y points:\n" << Y << std::endl;
  }

  double fit(double _x) {
    Eigen::VectorXd x(1);
    x(0) = _x;
    return MLS::fit(x);
  }
};

void TestFitSurface();
}; // namespace Algorithm
