#ifndef PATH_PLAN_DUBINS_H
#define PATH_PLAN_DUBINS_H

#include "matplotlibcpp.h"
#include <Eigen/Dense>
#include <algorithm>
#include <cmath>
#include <iostream>
#include <map>
#include <stdlib.h>
#include <time.h>
#include <vector>
namespace plt = matplotlibcpp;

using namespace std;
using namespace Eigen;

#define EPS 1e-4
#define PI 3.14159265354

class Dubins {
public:
  Dubins();
  ~Dubins();

  double mod2Pi(double theta);

  /// @brief [x,y]-->[r,\theta] polar
  /// @param x
  /// @param y
  /// @return
  vector<double> polar(double x, double y);

  /// @brief angle mod 2_pi
  /// @param angle
  /// @return
  double PI2PI(double angle);

  struct Path {
    // 车辆在起始圆上走过的圆弧长度为t,直线段长度为p,第二个圆上圆弧长度为q
    double t = -0., p = 0., q = 0.; //初始值给定一个不会出现的数
    // 总长度 = t+p+q
    string mode;
  };

  struct ResultDubins {
    vector<double> p_x;        // 所有插值点x
    vector<double> p_y;        // 所有插值点y
    vector<double> p_yaw;      // 所有插值点yaw
    vector<double> directions; // 所有插值点方向
    vector<double> lengths;    // 总路径长度/曲率
    string mode;
  };

public:
  Path left_straight_left(double alpha, double beta, double d);   // LSL
  Path right_straight_right(double alpha, double beta, double d); // RSR
  Path left_straight_right(double alpha, double beta, double d);  // LSR
  Path right_straight_left(double alpha, double beta, double d);  // RSL
  Path right_left_right(double alpha, double beta, double d);     // RSL
  Path left_right_left(double alpha, double beta, double d);      // LRL

  /// @brief 插值函数
  /// @param ind
  /// @param length
  /// @param m
  /// @param max_curvature
  /// @param origin_x
  /// @param origin_y
  /// @param origin_yaw
  /// @param path_x
  /// @param path_y
  /// @param path_yaw
  /// @param directions
  /// @return
  vector<vector<double>>
  interpolate(double ind, double length, char m, double max_curvature,
              double origin_x, double origin_y, double origin_yaw,
              vector<double> path_x, vector<double> path_y,
              vector<double> path_yaw, vector<double> directions);

  /// @brief
  /// @param total_length 圆弧+直线段的总长度
  /// @param lengths [path[t], path[p]，path[q]]
  /// @param modes best path mode
  /// @param max_curvature
  /// @param step_size
  /// @return
  vector<vector<double>>
  generate_local_course(double total_length, vector<double> lengths,
                        string modes, double max_curvature, double step_size);

  ResultDubins dubins_path_planning_from_origin(Vector3d goal, double curvature,
                                                double step_size);

  /// @brief dubins path planning
  /// @param start start pose [x,y,yaw]
  /// @param goal goal pose [x,y,yaw]
  /// @param curvature
  /// @param step_size
  /// @return
  ResultDubins dubins_path_planning(Vector3d start, Vector3d goal,
                                    double curvature, double step_size = 0.1);
};

#endif // PATH_PLAN_DUBINS_H