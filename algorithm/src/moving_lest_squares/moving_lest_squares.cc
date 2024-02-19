#include "moving_lest_squares/moving_lest_squares.h"
#include <random>

using namespace Algorithm;

MLS::MLS() {}

MLS::~MLS() {}

double MLS::fit(Eigen::VectorXd x) {
  int n = this->X_.size(); // points size

  Eigen::MatrixXd A(m_, m_);
  A.setZero();
  std::cout << "reset A(x):\n" << A << std::endl;

  Eigen::MatrixXd B(m_, n);
  B.setZero();
  std::cout << "reset B(x):\n" << B << std::endl;

  for (int i = 0; i < n; i++) {
    Eigen::VectorXd xi = X_[i];

    // 使用指数函数做权函数，入参是节点x与采样点xi的距离，权函数支持域尺寸
    double w = weightFunction((xi - x).norm(), h_);
    Eigen::VectorXd bxi = basis(xi);

    A += w * bxi * bxi.transpose();
    B.col(i) = w * bxi;
    std::cout << "id: " << i << " distance of x and xi: " << (xi - x).norm()
              << " weight w(x): " << w << " basis func:\n"
              << bxi << "\nbxi * bxi.transpose():\n"
              << bxi * bxi.transpose() << "\nupdate A:\n"
              << A << "\nupdate B:\n"
              << B << std::endl;
  }
  // solve A * ax = B*Y
  // f(x) = px^t^ * ax

  // 使用Eigen求解Ax=b中的x 近似解
  // A.colPivHouseholderQr().solve(b)
  Eigen::VectorXd ax = A.colPivHouseholderQr().solve(B * Y_);
  Eigen::VectorXd px = basis(x);
  std::cout << "coefficients ax:\n"
            << ax << "\ninput value basis:\n"
            << px.transpose() << std::endl;

  // 返回拟合函数 f(x)=p(x)^t^ * a(x)
  return px.transpose() * ax;
}

Eigen::VectorXd MLS::basis(const Eigen::VectorXd xi) {
  Eigen::VectorXd bx(m_);
  bx(0) = 1;
  bx(1) = xi(0);
  bx(2) = xi(0) * xi(0);
  return bx;
}

double MLS::weightFunction(double d, double h) {
  // Non-negative decaying function 非负衰减
  // Gaussian e^(-d*d/h*h)
  static double h2 = 1.0 / (h * h);
  return std::exp(-d * d * h2);

  // Wendland
  // static double inv_h = 1.0 / h;
  // return std::pow(1 - d * inv_h, 4) * (4.0 * d * inv_h + 1);

  // return 1.0 / (d * d + 0.0001);
}

MLSCurvedSurface::MLSCurvedSurface() : MLS() {
  // 维数2 多项式 size 6
  d_ = 2;
  m_ = 6;
}

MLSCurvedSurface::~MLSCurvedSurface() {}

Eigen::VectorXd MLSCurvedSurface::basis(const Eigen::VectorXd xi) {
  Eigen::VectorXd bx(m_);
  bx(0) = 1;
  bx(1) = xi(0);
  bx(2) = xi(1);
  bx(3) = xi(0) * xi(0);
  bx(4) = xi(0) * xi(1);
  bx(5) = xi(1) * xi(1);
  return bx;
}

MLSCurvedLine::MLSCurvedLine() : MLS() {}

MLSCurvedLine::~MLSCurvedLine() {}

void MLSCurvedLine::setX(std::vector<double> data_x) {
  X_ = std::vector<Eigen::VectorXd>(data_x.size(), Eigen::VectorXd{});
  for (int i = 0; i < data_x.size(); i++) {
    Eigen::VectorXd x(1);
    x(0) = data_x[i];
    X_[i] = x;
    std::cout << i << " x points:\n" << X_[i] << std::endl;
  }
}

void MLSCurvedLine::setY(std::vector<double> data_y) {
  Y_ = Eigen::VectorXd(data_y.size());
  for (int i = 0; i < data_y.size(); i++) {
    Y_(i) = data_y[i];
  }

  std::cout << "y points:\n" << Y_ << std::endl;
}

double MLSCurvedLine::fit(double input_value) {
  Eigen::VectorXd x(1);
  x(0) = input_value;
  return MLS::fit(x);
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
  mls.h_ = 0.1;
  mls.X_ = X;
  mls.Y_ = Y1;

  Eigen::VectorXd tx(2);
  tx(0) = -1.2;
  tx(1) = 2.3;
  double res = mls.fit(tx);
  std::cout << "test fit result: " << res << std::endl;

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
