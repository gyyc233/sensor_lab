#ifndef PATH_PLAN_RRT_STAR_H
#define PATH_PLAN_RRT_STAR_H

#include "rrt.h"

class RRT_Star : public RRT {
public:
  RRT_Star(const vector<vector<double>> &obstacleList,
           const vector<double> &randArea, const vector<double> &playArea,
           double robotRadius, double expandDis, double goalSampleRate,
           int maxIter, double connectCircleDist, bool searchUntilMaxIter);

  pair<vector<double>, vector<double>> planning();

  /// @brief 计算一定半径内所有的邻近节点
  /// @param new_node
  /// @return 所有邻近节点索引集合
  vector<int> findNearIndex(Node *new_node); //找出邻近节点集合

  /// @brief 计算起点传播到parent_node的cost总和
  /// @param parent_node
  void propagateCostToLeaves(Node *parent_node);

  /// @brief calculate cost
  /// @param from_node parent_node
  /// @param to_node sub_node
  /// @return cost
  double calcNewCost(Node *from_node, Node *to_node);

  /// @brief 布线
  /// @param new_node
  /// @param near_inds
  void rewire(Node *new_node, vector<int> near_indexes);

  /// @brief 计算离目标点的最佳索引
  /// @return
  int findBestGoalInd();

  /// @brief 为$x_{new}$重新选择父节点
  /// @param new_node
  /// @param near_indexes
  /// @return
  Node *chooseParent(Node *new_node, vector<int> near_indexes);

  double connect_circle_dist;
  bool search_until_max_iter;
};

#endif // PATH_PLAN_RRT_STAR_H
