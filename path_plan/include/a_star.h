#ifndef __PLAN_PATH_A_STAR_H__
#define __PLAN_PATH_A_STAR_H__

#include <Eigen/Dense>
#include <algorithm>
#include <cmath>
#include <iostream>
#include <map>
#include <stdlib.h>
#include <time.h>
#include <vector>

using namespace std;
using namespace Eigen;

#define EPS 1e-4
#define PI 3.14159265354

class Astar {
public:
  // 地图中的栅格点
  struct Node {
    double x;
    double y;
    float cost;
    // Node* p_node;
    double parent_index;

    Node(double x, double y, float cost, double parentIndex);
  };

private:
  double resolution; //栅格大小
  double robot_radius;
  double min_x, min_y, max_x, max_y; //地图范围
  double x_width, y_width;           //长宽
  vector<vector<bool>> obstacle_map; //障碍物地图
  vector<vector<double>> motion;     //障碍物地图
  vector<double> st, go;
  vector<double> ox, oy;

public:
  Astar(double resolution, double robotRadius);

  /**
   * 得到障碍物信息图，有障碍物的地方标记为true，没有标记为false
   * @param ox 障碍物x坐标集合
   * @param oy 障碍物y坐标集合
   */
  void calObstacleMap(const vector<double> &ox, const vector<double> &oy);

  /**
   * 计算栅格在地图中的位置
   * @param index
   * @param minp
   * @return
   */
  double calPosition(double index, double minp);

  /**
   * 标记各个方向的移动代价
   * @return
   */
  vector<vector<double>> getMotionModel();

  /**
   * 计算起点终点的栅格索引
   * @param position
   * @param minp
   * @return
   */
  double calXyIndex(double position, double minp);

  /**
   * 计算栅格索引
   * @param node
   * @return
   */
  double calIndex(Node *node);

  /**
   * 判断节点是否有效，即是否超出边界和碰到障碍物
   * @param node
   * @return
   */
  bool verifyNode(Node *node);

  /**
   * 计算路径，便于画图
   * @param goal_node
   * @param closed_set
   * @return
   */
  pair<vector<double>, vector<double>>
  calFinalPath(Node *goal_node, map<double, Node *> closed_set);

  /**
   * 规划
   * @param start 起点
   * @param goal 终点
   * @return 规划后的路径
   */
  pair<vector<double>, vector<double>> planning(vector<double> start,
                                                vector<double> goal);

  /**
   * 启发函数计算，与dijkstra不同的地方，astar增加了对目标点的启发函数
   * @param n1 节点1
   * @param n2 节点2
   * @return
   */
  double calHeuristic(Node *n1, Node *n2);

  /**
   * 画图
   * @param current
   */
  void plotGraph(Node *current);

  void setSt(const vector<double> &st);

  void setGo(const vector<double> &go);

  void setOx(const vector<double> &ox);

  void setOy(const vector<double> &oy);
};

#endif // __PLAN_PATH_A_STAR_H__
