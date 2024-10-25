
#include "dubins_curve.h"

int main() {
  Eigen::Vector3d start(1.0, 1.0, (double)45 / 180 * M_PI);
  Eigen::Vector3d end(1.0, 1.0, (double)45 / 180 * M_PI);
  double curvature = 1; // 曲率
  double step_size = 0.1;

  Dubins dubins;
  Dubins::ResultDubins rd =
      dubins.dubins_path_planning(start, end, curvature, step_size);
  
  return 0;
}
