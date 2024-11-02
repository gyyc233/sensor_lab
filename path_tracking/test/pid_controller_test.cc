#include "kinematic_model.h"
#include "matplotlibcpp.h"
#include "pid_controller.h"
#include <algorithm>
namespace plt = matplotlibcpp;

/**
 * PID控制器实现路径跟踪 位置式PID
 */

#define PI 3.1415926

/**
 * 得到距离参考轨迹最近点的下标
 * @param robot_state 机器人状态（x,y）
 * @param refer_path 参考路径
 * @return 距离参考轨迹最近点的下标
 */
double calTargetIndex(vector<double> robot_state,
                      vector<vector<double>> refer_path) {
  vector<double> dists;
  for (vector<double> xy : refer_path) {
    double dist =
        sqrt(pow(xy[0] - robot_state[0], 2) + pow(xy[1] - robot_state[1], 2));
    dists.push_back(dist);
  }
  return min_element(dists.begin(), dists.end()) -
         dists.begin(); // 返回vector最小元素的下标
}

int main() {
  vector<vector<double>> refer_path(1000, vector<double>(2));
  vector<double> refer_x, refer_y; // 保存参考数据用于画图
  // 生成参考轨迹
  for (int i = 0; i < 1000; i++) {
    refer_path[i][0] = 0.1 * i;
    refer_path[i][1] = 5 * sin(refer_path[i][0] / 3.0);
    refer_x.push_back(refer_path[i][0]);
    refer_y.push_back(refer_path[i][1]);
    //        cout<<refer_path[i][0]<<" ,"<<refer_path[i][1]<<endl;
  }

  // 运动学模型
  KinematicModel ugv(0, -1, 0.5, 1.5, 2, 0.1);

  // PID控制器，用PID限制转角误差
  PID_controller PID(3.0, 0.01, 30, 0., PI / 4, -PI / 4);
  // 保存机器人（小车）运动过程中的轨迹
  vector<double> x_, y_;
  // 机器人状态
  vector<double> robot_state(2);
  // 运行500个回合
  for (int i = 0; i < 800; i++) {
    plt::clf();
    robot_state[0] = ugv.x;
    robot_state[1] = ugv.y;
    // 参考博客中的公式
    double min_ind = calTargetIndex(robot_state, refer_path);
    cout << i << " robot_state: x: " << robot_state[0]
         << ", y:" << robot_state[1] << endl;
    cout << i << " refer_path: x: " << refer_path[min_ind][0]
         << ", y:" << refer_path[min_ind][1] << endl;
    // 计算当前位置和轨迹最近点的相对位姿
    // 朝向
    double alpha = atan2(refer_path[min_ind][1] - robot_state[1],
                         refer_path[min_ind][0] - robot_state[0]);
    // 距离
    double l_d = sqrt(pow(refer_path[min_ind][0] - robot_state[0], 2) +
                      pow(refer_path[min_ind][1] - robot_state[1], 2));

    if (sqrt(pow(refer_path[refer_path.size() - 1][0] - robot_state[0], 2) +
             pow(refer_path[refer_path.size() - 1][1] - robot_state[1], 2)) <
        0.1)
      break;

    // 计算偏航角
    double theta_e = alpha - ugv.psi;
    double e_y = -l_d * sin(theta_e); // 计算ugv.psi方向的距离
    double delta_f = PID.calOutput(e_y);
    std::cout << "PID error output: " << delta_f << std::endl;
    // 更新机器人状态
    ugv.updateState(0, delta_f); // accel =0 小车保持匀速，将PID输出作为转向角
    // 记录当前小车位置
    x_.push_back(ugv.x);
    y_.push_back(ugv.y);
    cout << i << " update car pose: x: " << ugv.x << ", y:" << ugv.y
         << ", psi: " << ugv.psi << ", v: " << ugv.v << endl;
    // 画图
    plt::plot(refer_x, refer_y, "b--");
    plt::plot(x_, y_, "r");
    plt::grid(true);
    plt::ylim(-8, 8);
    plt::pause(0.01);
  }
  // save figure
  const char *filename = "./pid_demo.png";
  cout << "Saving result to " << filename << std::endl;
  plt::save(filename);
  plt::show();
  return 0;
}