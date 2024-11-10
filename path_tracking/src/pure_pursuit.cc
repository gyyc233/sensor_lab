#include "pure_pursuit.h"

double PurePursuit::calTargetIndex(vector<double> robot_state,
                                   vector<vector<double>> refer_path,
                                   double l_d) {
  vector<double> dists;
  for (vector<double> xy : refer_path) {
    double dist =
        sqrt(pow(xy[0] - robot_state[0], 2) + pow(xy[1] - robot_state[1], 2));
    dists.push_back(dist);
  }
  double min_ind = min_element(dists.begin(), dists.end()) -
                   dists.begin(); //返回vector最小元素的下标

  double delta_l = sqrt(pow(refer_path[min_ind][0] - robot_state[0], 2) +
                        pow(refer_path[min_ind][1] - robot_state[1], 2));

  // 微调
  while (l_d > delta_l && min_ind < refer_path.size() - 1) {
    delta_l = sqrt(pow(refer_path[min_ind + 1][0] - robot_state[0], 2) +
                   pow(refer_path[min_ind + 1][1] - robot_state[1], 2));
    min_ind += 1;
  }
  return min_ind;
}

double PurePursuit::purePursuitControl(vector<double> robot_state,
                                       vector<double> current_ref_point,
                                       double l_d, double psi, double L) {
  // 带入 pure pursuit 控制量计算公式
  double alpha = atan2(current_ref_point[1] - robot_state[1],
                       current_ref_point[0] - robot_state[0]) -
                 psi;
  double delta = atan2(2 * L * sin(alpha), l_d);
  return delta;
}
