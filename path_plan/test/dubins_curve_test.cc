
#include "dubins_curve.h"

int main() {
  Eigen::Vector3d start(1.0, 1.0, (double)45 / 180 * M_PI);
  Eigen::Vector3d end(-2.0, 3.0, (double)-60 / 180 * M_PI);
  double curvature = 2; // 曲率
  double step_size = 0.1;

  Dubins dubins;
  Dubins::ResultDubins rd =
      dubins.dubins_path_planning(start, end, curvature, step_size);

  plt::plot(rd.p_x, rd.p_y);
  plt::plot(vector<double>{start[0]}, vector<double>{start[1]}, "og");
  plt::plot(vector<double>{end[0]}, vector<double>{end[1]}, "xb");
  plt::title("mode: " + rd.mode);

  const char *filename = "./dubins_demo.png";
  cout << "Saving result to " << filename << std::endl;
  plt::save(filename);
  plt::show();
  cout << "mode: " << rd.mode << endl;

  return 0;
}
