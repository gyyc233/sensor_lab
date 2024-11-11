#ifndef PATH_TRACKING_STANLEY_H
#define PATH_TRACKING_STANLEY_H
#include <algorithm>
#include <cmath>
#include <iostream>
#include <vector>
using namespace std;

#define PI 3.1415926

class Stanley {
private:
  double k;

public:
  Stanley(double k);

  /// @brief 搜索轨迹上最邻近目标点并返回索引
  /// @param robot_state 当前机器人位置
  /// @param refer_path 参考轨迹数组
  /// @return 最邻近目标点索引
  double calTargetIndex(vector<double> robot_state,
                        vector<vector<double>> refer_path);

  /// @brief 角度归一化到[-pi,pi]
  /// @param angle
  /// @return
  double normalizeAngle(double angle);

  /// @brief stanley控制
  /// @param robot_state 机器人位姿，包括x,y,yaw,v
  /// @param refer_path  参考轨迹的位置和参考轨迹上点的切线方向的角度 x,y,theta
  /// @return 控制量+目标点索引
  vector<double> stanleyControl(vector<double> robot_state,
                                vector<vector<double>> refer_path);
};

#endif // PATH_TRACKING_STANLEY_H
