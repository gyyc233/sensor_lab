#ifndef PATH_TRACKING_MY_REFERENCE_PATH
#define PATH_TRACKING_MY_REFERENCE_PATH

#include <Eigen/Dense>
#include <algorithm>
#include <cmath>
#include <iostream>
#include <vector>

using namespace std;
using namespace Eigen;

#define PI 3.1415926

struct refTraj {
  Eigen::MatrixXd xref, dref;
  int ind;
};

struct parameters {
  int L;
  int NX, NU, T;
  double dt;
};

class MyReferencePath {

public:
  // 生成默认的参考轨迹
  MyReferencePath();

  /**
  　* 计算最小跟踪误差
  　* @param robot_state  机器人状态
  　* @return 最小跟踪误差对应参考路径中的索引
　　*/
  std::vector<double> calcTrackError(std::vector<double> robot_state);

  /**
   * 角度归一化
   * @param angle
   * @return
   */
  double normalizeAngle(double angle);

  // 计算参考轨迹点，统一化变量数组，便于后面MPC优化使用.
  refTraj calc_ref_trajectory(std::vector<double> robot_state, parameters param,
                              double dl = 1.0);

public:
  // 生成的参考轨迹 refer_path包括4维：位置x, 位置y， 轨迹点的切线方向, 曲率k
  std::vector<std::vector<double>> refer_path;
  std::vector<double> refer_x, refer_y;
};

#endif // PATH_TRACKING_MY_REFERENCE_PATH
