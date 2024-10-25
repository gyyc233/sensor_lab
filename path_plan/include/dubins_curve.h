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
    double t = -0., p = 0., q = 0.; //初始值给定一个不会出现的数
    string mode;
  };

  struct ResultDubins {
    vector<double> p_x, p_y, p_yaw, directions, lengths;
    string mode;
  };

public:
  Path left_straight_left(double alpha, double beta, double d);   // LSL
  Path right_straight_right(double alpha, double beta, double d); // RSR
  Path left_straight_right(double alpha, double beta, double d);  // LSR
  Path right_straight_left(double alpha, double beta, double d);  // RSL
  Path right_left_right(double alpha, double beta, double d);     // RSL
  Path left_right_left(double alpha, double beta, double d);      // LRL

  vector<vector<double>>
  interpolate(double ind, double length, char m, double max_curvature,
              double origin_x, double origin_y, double origin_yaw,
              vector<double> path_x, vector<double> path_y,
              vector<double> path_yaw, vector<double> directions);

  vector<vector<double>>
  generate_local_course(double total_length, vector<double> lengths,
                        string modes, double max_curvature, double step_size);

  ResultDubins dubins_path_planning_from_origin(Vector3d goal, double curvature,
                                                double step_size);

  ResultDubins dubins_path_planning(Vector3d start, Vector3d goal,
                                    double curvature, double step_size = 0.1);
};

#endif // PATH_PLAN_DUBINS_H