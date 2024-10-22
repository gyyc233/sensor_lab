#ifndef PATH_PLAN_B_SPLINE_H
#define PATH_PLAN_B_SPLINE_H
#include <Eigen/Dense>
#include <algorithm>
#include <cmath>
#include <iostream>
#include <vector>
using namespace std;
using namespace Eigen;

/// @brief 基函数定义
/// @param i
/// @param k B 样条阶数k
/// @param u 自变量
/// @param node_vector 节点向量 array([u0,u1,u2,...,u_n+k],shape=[1,n+k+1]
/// @return
double baseFunction(int i, int k, double u, vector<double> node_vector);

/// @brief 准均匀B样条的节点向量计算, 首末值定义为 0 和 1
/// @param n 控制点个数-1，控制点共n+1个
/// @param k B样条阶数k， k阶B样条，k-1次曲线
/// @return
vector<double> u_quasi_uniform(int n, int k);

/// @brief 分段B样条, 首末值定义为 0 和 1
/// @details 分段Bezier曲线的节点向量计算，共n+1个控制顶点，k阶B样条，k-1次曲线,
/// 分段Bezier端节点重复度为k，内间节点重复度为k-1,且满足n/(k-1)为正整数
/// @param n 控制点个数-1，控制点共n+1个
/// @param k B样条阶数k， k阶B样条，k-1次曲线
/// @return
vector<double> u_piecewise_B_Spline(int n, int k);

#endif // PATH_PLAN_B_SPLINE_H
