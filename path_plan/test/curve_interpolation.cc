#include "iostream"
#include "matplotlibcpp.h"
#include "vector"
#include <Eigen/Dense>

namespace plt = matplotlibcpp;

int main() {
  //换道场景路段与车辆相关参数的定义
  double d = 3.5;       //道路标准宽度
  double len_line = 30; //直线段长度
  double W = 1.75;      //车宽
  double L = 4.7;       //车长

  //车辆换道初始状态与终点期望状态
  double t0 = 0, t1 = 3;
  //分别表示小车的x,y; vx,vy; ax,ay
  Eigen::VectorXd state_t0(6), state_t1(6);
  state_t0 << 0, -d / 2, 5, 0, 0, 0;
  state_t1 << 20, d / 2, 5, 0, 0, 0;

  //把起末两点的横纵向方程统一用矩阵表达
  Eigen::VectorXd X(6), Y(6);
  X << state_t0[0], state_t0[2], state_t0[4], state_t1[0], state_t1[2],
      state_t1[4];
  Y << state_t0[1], state_t0[3], state_t0[5], state_t1[1], state_t1[3],
      state_t1[5];

  Eigen::MatrixXd T(6, 6);
  T << pow(t0, 5), pow(t0, 4), pow(t0, 3), pow(t0, 2), t0, 1, 5 * pow(t0, 4),
      4 * pow(t0, 3), 3 * pow(t0, 2), 2 * t0, 1, 0, 20 * pow(t0, 3),
      12 * pow(t0, 3), 6 * t0, 1, 0, 0, pow(t1, 5), pow(t1, 4), pow(t1, 3),
      pow(t1, 2), t1, 1, 5 * pow(t1, 4), 4 * pow(t1, 3), 3 * pow(t1, 2), 2 * t1,
      1, 0, 20 * pow(t1, 3), 12 * pow(t1, 3), 6 * t1, 1, 0, 0;

  //计算A和B两个系数矩阵
  Eigen::MatrixXd A = T.inverse() * X;
  Eigen::MatrixXd B = T.inverse() * Y;

  std::cout << A << std::endl;
  std::cout << B << std::endl;

  // draw

  // 把t0--t1 时间段进行均分
  std::vector<double> time;
  int cnt = 0;
  for (double t = t0; t < t1 + 0.1; t += 0.1) {
    cnt++;
    time.push_back(t);
  }

  std::vector<double> x_, y_, v_lon, v_lat, a_lon,
      a_lat; //用于保存数据，横纵向位置、横纵向速度
  Eigen::MatrixXd temp1(1, 6), temp2(1, 6), temp3(1, 6);
  for (int i = 0; i < cnt; i++) {
    temp1 << pow(time[i], 5), pow(time[i], 4), pow(time[i], 3), pow(time[i], 2),
        time[i], 1;
    x_.push_back((temp1 * A)(0, 0));
    y_.push_back((temp1 * B)(0, 0));

    temp2 << 5 * pow(time[i], 4), 4 * pow(time[i], 3), 3 * pow(time[i], 2),
        2 * time[i], 1, 0;
    v_lon.push_back((temp2 * A)(0, 0));
    v_lat.push_back((temp2 * B)(0, 0));

    temp3 << 20 * pow(time[i], 3), 12 * pow(time[i], 2), 6 * time[i], 2, 0, 0;
    a_lon.push_back((temp3 * A)(0, 0));
    a_lat.push_back((temp3 * B)(0, 0));
  }

  //画图
  plt::figure(1);
  plt::plot(x_, y_, "r");

  // 横向速度
  plt::figure(2);
  plt::plot(time, v_lon);
  // 纵向速度
  plt::figure(3);
  plt::plot(time, v_lat);

  // 横向加速度
  plt::figure(4);
  plt::plot(time, a_lon);

  plt::figure(5);
  plt::plot(time, a_lat);
  plt::show();

  return 0;
}
