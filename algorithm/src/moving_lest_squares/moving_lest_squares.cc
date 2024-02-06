#include "moving_lest_squares/moving_lest_squares.h"
#include <random>

using namespace Algorithm;

double MLS::fit(Eigen::VectorXd x) {
  int n = this->X.size();

  Eigen::MatrixXd A(m, m);
  A.setZero();
  Eigen::MatrixXd B(m, n);
  B.setZero();

  for (int i = 0; i < n; i++) {
    Eigen::VectorXd xi = X[i];
    double w = wf((xi - x).norm(), h);

    Eigen::VectorXd bxi = basis(xi);
    A += w * bxi * bxi.transpose();
    B.col(i) = w * bxi;
  }
  // solve A * cx = B*Y
  // f(x) = bx*cx
  Eigen::VectorXd cx = A.colPivHouseholderQr().solve(B * Y);
  Eigen::VectorXd bx = basis(x);

  return bx.transpose() * cx;
}

void TestFitSurface() {
  auto TestFunc = [](double x, double y) {
    double z =
        2 * (1 - x) * (1 - x) * std::exp(-x * x - (y + 1) * (y + 1)) -
        10 * (x * x * x * x / 5 - std::pow(y, 5)) * std::exp(-x * x - y * y) -
        1.0 / 3 * std::exp(-(x + 1) * (x + 1) - y * y);
    return z;
  };

  const int n = 400;
  std::vector<Eigen::VectorXd> X(n, Eigen::VectorXd{2});
  Eigen::VectorXd Y1(n);

  std::default_random_engine generator(time(NULL));
  std::uniform_real_distribution<double> distribution(-3, 3);
  std::mt19937 dev(123);
  distribution(dev);
  for (int i = 0; i < n; i++) {
    X[i](0) = distribution(dev);
    X[i](1) = distribution(dev);
    Y1(i) = TestFunc(X[i](0), X[i](1));
  }

  MLSCurvedSurface mls;
  mls.h = 0.1;
  mls.X = X;
  mls.Y = Y1;

  Eigen::VectorXd tx(2);
  tx(0) = -1.2;
  tx(1) = 2.3;
  double res = mls.fit(tx);

  /*using namespace matplot;
  auto [_X, _Y] = meshgrid(iota(-3, .1, +3));
  auto _Z = transform(_X, _Y, [&](double x, double y)
          {
                  Eigen::VectorXd tx(2);
                  tx(0) = x;
                  tx(1) = y;
                  return mls.fit(tx);
          });
  mesh(_X, _Y, _Z);

  show();*/
}
