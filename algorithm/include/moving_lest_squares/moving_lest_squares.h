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
  MLS();

  virtual ~MLS();

  /// @brief MLS fit
  /// @param x input value
  /// @return fitted value
  virtual double fit(Eigen::VectorXd x);

  /// @brief built basis
  /// @param xi basis p(x)=[1,x,x*x]
  /// @return basis
  virtual Eigen::VectorXd basis(const Eigen::VectorXd xi);

  /// @brief calculate gaussian kernel weight function, expected value: 0,
  /// variance value: h
  /// @param d x value
  /// @param h variance
  /// @return weight function value
  virtual double weightFunction(double d, double h);

public:
  std::vector<Eigen::VectorXd> X_;
  Eigen::VectorXd Y_;

public:
  // h is a spacing parameter which can be used to smooth out
  // small features in the data, see[Levin 2003; Alexa et al. 2003].
  // h 是一个间距参数，可用于平滑数据中的小特征
  double h_ = 0.1;

  // d spatial dimensions 空间尺寸,一维是1,二维是2,三维3
  int d_ = 1;

  // The number of terms in a polynomial
  // In paper <<An As-Short-As-Possible Introduction to the Least Squares,
  // Weighted Least Squaresand Moving Least Squares Methods for Scattered Data
  // Approximationand Interpolation>> m represents total degree
  // 基函数多项式的项数
  int m_ = 3;
};

//--------------------------------------------------------------------------------------------------
//
// Wrapper of MLS 包装器
//
//--------------------------------------------------------------------------------------------------

/// @brief 曲线MLS拟合
class MLSCurvedSurface : public MLS {
public:
  MLSCurvedSurface();

  ~MLSCurvedSurface();

  /// @brief built basis
  /// @param xi basis p(x)=[1,x,y,xy,x*x,y*y]
  /// @return basis
  Eigen::VectorXd basis(const Eigen::VectorXd xi) override;
};

/// @brief 曲线MSL拟合
class MLSCurvedLine : public MLS {
public:
  MLSCurvedLine();

  ~MLSCurvedLine();

  /// @brief
  /// @param data_x
  void setX(std::vector<double> data_x);

  void setY(std::vector<double> data_y);

  double fit(double input_value);
};

void TestFitSurface();
}; // namespace Algorithm
