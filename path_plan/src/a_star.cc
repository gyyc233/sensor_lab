#include "a_star.h"
#include "matplotlibcpp.h"

namespace plt = matplotlibcpp;

Astar::Node::Node(double x, double y, float cost, double parentIndex)
    : x(x), y(y), cost(cost), parent_index(parentIndex) {}

Astar::Astar(double resolution, double robotRadius)
    : resolution(resolution), robot_radius(robotRadius) {}

void Astar::calObstacleMap(const vector<double> &ox, const vector<double> &oy) {
  min_x = round(*min_element(ox.begin(), ox.end()));
  min_y = round(*min_element(oy.begin(), oy.end()));
  max_x = round(*max_element(ox.begin(), ox.end()));
  max_y = round(*max_element(oy.begin(), oy.end()));

  cout << "min_x:" << min_x << "   min_y:" << min_y << "  max_x:" << max_x
       << "  max_y:" << max_y << endl;

  // 计算栅格数量
  x_width = round((max_x - min_x) / resolution);
  y_width = round((max_y - min_y) / resolution);
  cout << "grid sum x_width:" << x_width << "  y_width:" << y_width << endl;

  // init grid map
  obstacle_map = vector<vector<bool>>(x_width, vector<bool>(y_width, false));

  // mark occupy
  for (double i = 0; i < x_width; i++) {
    double x = calPosition(i, min_x);
    for (double j = 0; j < y_width; j++) {
      double y = calPosition(j, min_y);
      for (double k = 0; k < ox.size(); k++) {
        double d = sqrt(pow(ox[k] - x, 2) + pow(oy[k] - y, 2));
        if (d <= robot_radius) {
          obstacle_map[i][j] = true;
          break;
        }
      }
    }
  }
}

double Astar::calPosition(double index, double minp) {
  double pos = index * resolution + minp;
  return pos;
}

vector<vector<double>> Astar::getMotionModel() {
  // x,y,cost
  // 说明向8个方向移动所需要的代价
  motion = {{1, 0, 1},        {0, 1, 1},         {-1, 0, 1},
            {0, -1, 1},       {-1, -1, sqrt(2)}, {-1, 1, sqrt(2)},
            {1, -1, sqrt(2)}, {1, 1, sqrt(2)}};
  return motion;
}

double Astar::calXyIndex(double position, double minp) {
  return round((position - minp) / resolution);
}

double Astar::calIndex(Astar::Node *node) {
  // cout<<node->x<<","<<node->y<<endl;
  return (node->y - min_y) * x_width + (node->x - min_x);
}

bool Astar::verifyNode(Astar::Node *node) {
  double px = calPosition(node->x, min_x);
  double py = calPosition(node->y, min_y);
  if (px < min_x)
    return false;
  if (py < min_y)
    return false;
  if (px >= max_x)
    return false;
  if (py >= max_y)
    return false;
  if (obstacle_map[node->x][node->y])
    return false;
  return true;
}

pair<vector<double>, vector<double>>
Astar::calFinalPath(Astar::Node *goal_node, map<double, Node *> closed_set) {
  vector<double> rx, ry;
  rx.push_back(calPosition(goal_node->x, min_x));
  ry.push_back(calPosition(goal_node->y, min_y));

  double parent_index = goal_node->parent_index;

  while (parent_index != -1) {
    Node *node = closed_set[parent_index];
    rx.push_back(calPosition(node->x, min_x));
    ry.push_back(calPosition(node->y, min_y));

    parent_index = node->parent_index;
  }
  return {rx, ry};
}

pair<vector<double>, vector<double>> Astar::planning(vector<double> start,
                                                     vector<double> goal) {
  double sx = start[0], sy = start[1];
  double gx = goal[0], gy = goal[1];
  // cost 置0
  Node *start_node =
      new Node(calXyIndex(sx, min_x), calXyIndex(sy, min_y), 0.0, -1);
  Node *goal_node =
      new Node(calXyIndex(gx, min_x), calXyIndex(gy, min_y), 0.0, -1);

  map<double, Node *> open_set, closed_set;
  //将起点加入到open set
  open_set[calIndex(start_node)] = start_node;
  // cout<<calIndex(start_node)<<endl;
  Node *current;
  while (true) {
    double c_id = numeric_limits<double>::max();
    double cost = numeric_limits<double>::max();
    //计算代价最小的节点,与dijkstra代码维一不同的地方，即启发函数的计算不同，其他都一样
    for (auto it = open_set.begin(); it != open_set.end(); it++) {
      // f(n)=g(n)+h(n)
      double now_cost = it->second->cost + calHeuristic(goal_node, it->second);
      if (now_cost < cost) {
        cost = now_cost;
        c_id = it->first;
      }
      // cout<<it->first<<","<<it->second->cost<<endl;
    }
    current = open_set[c_id];

    plotGraph(current); //画图

    // 如果 current node 是终点
    if (abs(current->x - goal_node->x) < EPS &&
        abs(current->y - goal_node->y) < EPS) {
      cout << "Find goal" << endl;
      goal_node->parent_index = current->parent_index;
      goal_node->cost = current->cost;
      break;
    }

    //从open set中去除并加入closed set
    auto iter = open_set.find(c_id);
    open_set.erase(iter);
    //将其加入到closed set
    closed_set[c_id] = current;

    // expand search grid based on motion model
    // 遍历节点附近邻域
    for (vector<double> move : motion) {
      // cout<<move[0]<<move[1]<<move[2]<<endl;
      Node *node = new Node(current->x + move[0], current->y + move[1],
                            current->cost + move[2], c_id);
      double n_id = calIndex(node);

      if (closed_set.find(n_id) != closed_set.end())
        continue; //如果已经在closed_set中则跳过

      if (!verifyNode(node))
        continue; //如果超出边界或者碰到障碍物，跳过

      if (open_set.find(n_id) == open_set.end()) { //如果open set中没有这个节点
        open_set[n_id] = node;
      } else { //如果open set中已经存在这个节点
        if (open_set[n_id]->cost >= node->cost) {
          open_set[n_id] = node;
        }
      }
    }
  }
  return calFinalPath(goal_node, closed_set);
}

void Astar::plotGraph(Astar::Node *current) {
  // plt::clf();
  plt::plot(ox, oy, ".k");
  plt::plot(vector<double>{st[0]}, vector<double>{st[1]}, "og");
  plt::plot(vector<double>{go[0]}, vector<double>{go[1]}, "xb");
  plt::grid(true);
  plt::plot(vector<double>{calPosition(current->x, min_x)},
            vector<double>{calPosition(current->y, min_y)}, "xc");
  plt::pause(0.001);
}

void Astar::setSt(const vector<double> &st) { Astar::st = st; }

void Astar::setGo(const vector<double> &go) { Astar::go = go; }

void Astar::setOx(const vector<double> &ox) { Astar::ox = ox; }

void Astar::setOy(const vector<double> &oy) { Astar::oy = oy; }

double Astar::calHeuristic(Astar::Node *n1, Astar::Node *n2) {
  double w = 1.0; //启发函数权重
  // double d = w * sqrt(pow(n1->x - n2->x, 2) + pow(n1->y - n2->y, 2)); //
  // 欧氏距离
  double d = w * (abs(n1->x - n2->x) + abs(n1->y - n2->y)); // 曼哈顿距离
  return d;
}
