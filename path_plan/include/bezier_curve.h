#ifndef PATH_PLAN_BEZIERCURVE_H
#define PATH_PLAN_BEZIERCURVE_H

#include <Eigen/Dense>
#include <algorithm>
#include <cmath>
#include <iostream>
#include <vector>
using namespace std;
using namespace Eigen;

double factorial(int n);

Vector2d bezierCommon(vector<Vector2d> Ps, double t);

Vector2d bezierRecursion(vector<Vector2d> Ps, double t);

#endif // PATH_PLAN_BEZIERCURVE_H
