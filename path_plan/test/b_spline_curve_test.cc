#include "b_spline_curve.h"
#include "matplotlibcpp.h"

namespace plt = matplotlibcpp;

int main() {
  vector<Vector2d> Ps{Vector2d(9.0, 10.5),  Vector2d(51.0, 40.23),
                      Vector2d(77.0, 30.0), Vector2d(65.0, 15.8),
                      Vector2d(40.0, 10.0), Vector2d(25.3, 15.8)};

  vector<double> x_ref, y_ref;
  for (int i = 0; i < Ps.size(); i++) {
    x_ref.push_back(Ps[i][0]);
    y_ref.push_back(Ps[i][1]);
  }
  vector<double> x_, y_;

  int n = Ps.size() - 1; //控制点个数-1
  int k = 3;             // k阶、k-1次B样条
  Vector2d p_u(0, 0);
  vector<double> bik_u(n + 1);

  int flag;
  cout << "请选择:1. 均匀B样条 2.准均匀B样条  3.分段B样条 0. 退出 " << endl;
  cin >> flag;
  vector<double> node_vector;
  switch (flag) {
  case 1: //均匀B样条
    for (int i = 0; i < n + k + 1; i++) {
      node_vector.push_back((double)i / (n + k + 1));
    }

    break;
  case 2:
    node_vector = u_quasi_uniform(n, k);

    break;
  case 3:
    node_vector = u_piecewise_B_Spline(n, k);

    break;
  default:
    return 0;
  }
  if (flag == 1) {
    for (double u = (double)(k - 1) / (n + k + 1);
         u < (double)(n + 2) / (n + k + 1); u += 0.005) {
      for (int i = 0; i < n + 1; i++) {
        bik_u[i] = baseFunction(i, k, u, node_vector);
        // cout<<bik_u[i]<<endl;
      }
      for (int i = 0; i < Ps.size(); i++) {
        p_u = p_u + Ps[i] * bik_u[i];
      }
      // cout<<p_u<<endl;
      x_.push_back(p_u[0]);
      y_.push_back(p_u[1]);
      p_u = Vector2d(0, 0);
    }
  } else {
    for (double u = 0; u < 1; u += 0.005) {
      for (int i = 0; i < n + 1; i++) {
        bik_u[i] = baseFunction(i, k, u, node_vector);
      }
      for (int i = 0; i < Ps.size(); i++) {
        p_u = p_u + Ps[i] * bik_u[i];
        // cout<<p_u<<","<<endl;
      }
      x_.push_back(p_u[0]);
      y_.push_back(p_u[1]);
      p_u = Vector2d(0, 0);
    }
  }
  std::cout << "x_ size: " << x_.size() << ", " << x_[0] << std::endl;
  std::cout << "y_ size: " << y_.size() << ", " << y_[0] << std::endl;

  //画图
  // plt::xlim(0,1);
  plt::plot(x_, y_, "r");
  plt::plot(x_ref, y_ref);
  plt::pause(0.01);
  // save figure
  const char *filename = "./b_spline_demo.png";
  cout << "Saving result to " << filename << std::endl;
  plt::save(filename);
  plt::show();
  return 0;
}
