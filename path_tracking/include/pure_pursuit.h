#ifndef PATH_TRACKING_PUREPURSUIT_H
#define PATH_TRACKING_PUREPURSUIT_H
#include <algorithm>
#include <cmath>
#include <iostream>
#include <vector>

using namespace std;
class PurePursuit {
public:
  /// @brief 计算邻近路点
  /// @param robot_state 当前机器人位置
  /// @param refer_path 参考轨迹组
  /// @param l_d 前向距离
  /// @return 邻近路点在参考轨迹组中的索引
  double calTargetIndex(vector<double> robot_state,
                        vector<vector<double>> refer_path, double l_d);

  /// @brief pure pursuit
  /// @param robot_state current robot pose
  /// @param current_ref_point reference tracking point
  /// @param l_d forward distance
  /// @param psi 机器人航向角
  /// @param L 轴距
  /// @return 转角控制量
  double purePursuitControl(vector<double> robot_state,
                            vector<double> current_ref_point, double l_d,
                            double psi, double L);
};

#endif // PATH_TRACKING_PUREPURSUIT_H
