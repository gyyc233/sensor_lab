#ifndef PATH_PLAN_RRT_H
#define PATH_PLAN_RRT_H

#include "matplotlibcpp.h"
#include <Eigen/Dense>
#include <algorithm>
#include <cmath>
#include <iostream>
#include <stdlib.h>
#include <time.h>
#include <vector>
namespace plt = matplotlibcpp;

using namespace std;
using namespace Eigen;

#define PI 3.14159265354

class RRT {
public:
  struct Node {
    double x, y;                             //节点坐标
    vector<double> path_x = {}, path_y = {}; // 路径，作为画图的数据
    Node(double x, double y);
    Node *parent;
    double cost;
  };

public:
  vector<vector<double>>
      obstacle_list; //障碍物位置列表
                     //[[x,y,size],...]，这里用圆形障碍物，容易判断线段是否与障碍相交
  vector<double> rand_area; //采样区域 x,y ∈ [min,max];
  vector<double> play_area; //约束随机树的范围[xmin,xmax,ymin,ymax]

  double robot_radius;     //机器人半径
  double expand_dis;       //扩展的步长
  double goal_sample_rate; //采样目标点的概率，百分制.default:
                           // 5，即表示5%的概率直接采样目标点
  vector<Node *> node_list;
  Node *begin; //根节点
  Node *end;   //终节点

  int max_iter;

public:
  /// @brief
  /// @param obstacleList 障碍物位置列表
  /// @param randArea 采样区域
  /// @param playArea 约束随机树的范围
  /// @param robotRadius 机器人半径
  /// @param expandDis 拓展半径
  /// @param goalSampleRate 目标点采样概率
  /// @param maxIter 最大迭代次数
  RRT(const vector<vector<double>> &obstacleList,
      const vector<double> &randArea, const vector<double> &playArea,
      double robotRadius, double expandDis, double goalSampleRate, int maxIter);

  vector<double>
  calDistanceAngle(Node *from_node,
                   Node *to_node); // 计算两个节点间的距离和方位角

  /// @brief 判断node与其它点连线是否有障碍
  /// @param node
  /// @return if safety return true
  bool obstacleFree(Node *node); //判断是否有障碍物

  bool isInsidePlayArea(Node *node); //判断是否在可行区域里面

  int getNearestNodeIndex(vector<Node *> node_list,
                          Node *rnd_node); //计算最近的节点

  Node *sampleFree(); //采样生成节点

  double calDistToGoal(double x, double y); //计算(x,y)离目标点的距离

  pair<vector<double>, vector<double>>
  generateFinalCourse(double goal_ind); //生成路径，画图

  Node *
  steer(Node *from_node, Node *to_node,
        double extend_length =
            numeric_limits<double>::max()); //连线方向扩展固定步长查找x_new

  pair<vector<double>, vector<double>> planning();

  void setBegin(Node *begin);

  void setAnEnd(Node *anEnd);

  void plotCircle(double x, double y, double size, string color = "b"); //画圆
  void draw(Node *node = nullptr);
};

#endif // PATH_PLAN_RRT_H
