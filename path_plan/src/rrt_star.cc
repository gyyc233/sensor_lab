#include "rrt_star.h"

RRT_Star::RRT_Star(const vector<vector<double>> &obstacleList,
                   const vector<double> &randArea,
                   const vector<double> &playArea, double robotRadius,
                   double expandDis, double goalSampleRate, int maxIter,
                   double connectCircleDist, bool searchUntilMaxIter)
    : RRT(obstacleList, randArea, playArea, robotRadius, expandDis,
          goalSampleRate, maxIter),
      connect_circle_dist(connectCircleDist),
      search_until_max_iter(searchUntilMaxIter) {}

pair<vector<double>, vector<double>> RRT_Star::planning() {
  // 将起点加入随机树集合中
  node_list.push_back(begin);
  for (int i = 0; i < max_iter; i++) {
    Node *random_node = sampleFree();
    std::cout << "random tree node size: " << node_list.size() << std::endl;

    // 已生成的树中利用欧氏距离判断距离x_{rand}最近的点x_{near}
    int nearest_id = getNearestNodeIndex(node_list, random_node);
    Node *nearest_node = node_list[nearest_id];

    // 从x_{near}与x_{rand}的连线方向上扩展固定步长u，得到新节点 x_{new}
    Node *new_node = steer(nearest_node, random_node, expand_dis);

    // 计算new_node cost
    new_node->cost =
        nearest_node->cost + sqrt(pow(new_node->x - nearest_node->x, 2) +
                                  pow(new_node->y - nearest_node->y, 2));

    // 如果在可行区域内且x_{near}与x_{new}之间无障碍物
    if (isInsidePlayArea(new_node) && obstacleFree(new_node)) {
      vector<int> near_ids = findNearIndex(new_node);
      //　为x_{new}重新选择父节点
      Node *node_with_updated_parent = chooseParent(new_node, near_ids);
      // 如果父节点更新了
      if (node_with_updated_parent) {
        // 重布线
        rewire(node_with_updated_parent, near_ids);
        node_list.push_back(node_with_updated_parent);
      } else {
        node_list.push_back(new_node);
      }
    }

    draw(random_node);
    // reaches goal
    if ((!search_until_max_iter) && new_node) {
      int last_index = findBestGoalInd();
      if (last_index != -1) {
        std::cout << "reached the goal!" << std::endl;
        return generateFinalCourse(last_index);
      }
    }
  }

  std::cout << "reached the maximum iteration" << std::endl;
  int last_index = findBestGoalInd();
  if (last_index != -1) {
    return generateFinalCourse(last_index);
  }

  return {};
}

vector<int> RRT_Star::findNearIndex(RRT::Node *new_node) {
  int num_node = node_list.size() + 1;
  vector<int> indexes;
  std::cout << "num_node: " << num_node << std::endl;
  std::cout << "log(num_node): " << log(num_node) << std::endl;
  std::cout << "log(num_node) / num_node: " << log(num_node) << std::endl;
  std::cout << "connect_circle_dist: " << connect_circle_dist << std::endl;
  double r = connect_circle_dist * sqrt(log(num_node) / num_node);
  std::cout << "r: " << r << std::endl;

  for (int i = 0; i < node_list.size(); i++) {
    if (pow(node_list[i]->x - new_node->x, 2) +
            pow(node_list[i]->y - new_node->y, 2) <
        r * r) {
      indexes.push_back(i);
    }
  }

  return indexes;
}

void RRT_Star::propagateCostToLeaves(RRT::Node *parent_node) {
  for (int i = 0; i < node_list.size(); i++) {
    if (node_list[i]->parent == parent_node) {
      node_list[i]->cost = calcNewCost(parent_node, node_list[i]);
      propagateCostToLeaves(node_list[i]);
    }
  }
}

double RRT_Star::calcNewCost(Node *from_node, Node *to_node) {
  vector<double> distance_with_angle = calDistanceAngle(from_node, to_node);
  return from_node->cost + distance_with_angle[0];
}

void RRT_Star::rewire(RRT::Node *new_node, vector<int> near_indexes) {
  for (int i : near_indexes) {
    Node *near_node = node_list[i];
    Node *edge_node = steer(new_node, near_node);
    if (!edge_node)
      continue;

    edge_node->cost = calcNewCost(new_node, near_node);
    // edge_node 与其余相邻点连线都没有障碍且cost小于原先的near_node,更换
    if (obstacleFree(edge_node) && near_node->cost > edge_node->cost) {
      near_node->x = edge_node->x;
      near_node->y = edge_node->y;
      near_node->cost = edge_node->cost;
      near_node->path_x = edge_node->path_x;
      near_node->path_y = edge_node->path_y;
      near_node->parent = near_node->parent;

      // 更新cost
      propagateCostToLeaves(new_node);
    }
  }
}

int RRT_Star::findBestGoalInd() {
  vector<int> goal_indexes;
  for (int i = 0; i < node_list.size(); i++) {
    double distance = calDistToGoal(node_list[i]->x, node_list[i]->y);
    if (distance <= expand_dis) {
      goal_indexes.push_back(i);
    }
  }

  vector<int> safe_goal_indexes;
  for (int i = 0; i < goal_indexes.size(); i++) {
    Node *node = node_list[goal_indexes[i]];
    steer(node, end);
    if (obstacleFree(node)) {
      safe_goal_indexes.push_back(goal_indexes[i]);
    }
  }

  if (safe_goal_indexes.empty())
    return -1;

  double min_cost = numeric_limits<double>::max();
  int safe_index = -1;
  for (int id : safe_goal_indexes) {
    if (node_list[id]->cost < min_cost) {
      min_cost = node_list[id]->cost;
      safe_index = id;
    }
  }

  return safe_index;
}

RRT::Node *RRT_Star::chooseParent(Node *new_node, vector<int> near_indexes) {
  // 在新产生的节点 $x_{new}$ 附近以定义的半径范围$r$内寻找所有的近邻节点
  // $X_{near}$, 作为替换 $x_{new}$ 原始父节点 $x_{near}$ 的备选
  vector<double> costs;
  for (int i : near_indexes) {
    Node *near_node = node_list[i];
    //　TODO: 为什么要再次采样，而不是之间直接calcNewCost(near_node, new_node)
    Node *t_node = steer(near_node, new_node);
    // 判断t_node与之前所有点之间连线是否有障碍
    if (t_node && obstacleFree(t_node)) {
      // 需要依次计算起点到每个近邻节点 $X_{near}$ 的路径代价, 加上近邻节点
      // $X_{near}$ 到 $x_{new}$ 的路径代价
      double cost = calcNewCost(near_node, new_node);
      costs.push_back(cost);
    } else {
      costs.push_back(std::numeric_limits<double>::max()); // collision node
    }
  }
  if (costs.empty())
    return nullptr;

  // 取路径代价最小的近邻节点$x_{min}$作为 $x_{new}$ 新的父节点
  double min_cost = *std::min_element(costs.begin(), costs.end());
  if (min_cost == std::numeric_limits<double>::max()) {
    std::cout << "there is no good path" << std::endl;
    return nullptr;
  }

  // get min cost indexes
  double min_cost_index =
      near_indexes[std::min_element(costs.begin(), costs.end()) -
                   costs.begin()];

  Node *determine_node = steer(node_list[min_cost_index], new_node);
  determine_node->cost = min_cost;
  return determine_node;
}
