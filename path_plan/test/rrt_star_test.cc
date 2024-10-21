#include "rrt_star.h"

int main() {
  std::vector<std::vector<double>> obstacle_list{
      {5, 5, 1}, {3, 6, 2}, {3, 8, 2},  {3, 10, 2},
      {7, 5, 2}, {9, 5, 2}, {8, 10, 1}, {6, 12, 1}};

  RRT::Node *begin_node = new RRT::Node(0.0, 0.0);
  RRT::Node *end_node = new RRT::Node(6.0, 10.0);

  std::vector<double> rnd_area{-2, 15}; // 采样区域，x,y ∈ [min,max];
  std::vector<double> play_area{-1, 12, 0,
                                14}; // 约束随机树的范围[xmin,xmax,ymin,ymax]

  double radius = 0.5;
  double expand_dist = 3; // 拓展的步长
  double goal_sample_rate =
      20; //采样目标点的概率，百分制.default: 5，即表示5%的概率直接采样目标点
  int max_iter = 500;
  double connect_circle_dist = 5.0;
  bool search_until_max_iter = false;

  RRT_Star rrt(obstacle_list, rnd_area, play_area, radius, expand_dist,
               goal_sample_rate, max_iter, connect_circle_dist,
               search_until_max_iter);
  rrt.setBegin(begin_node);
  rrt.setAnEnd(end_node);

  std::pair<std::vector<double>, std::vector<double>> trajectory =
      rrt.planning();

  plt::plot(trajectory.first, trajectory.second, "r");
  // save figure
  const char *filename = "./rrt_star_demo.png";
  cout << "Saving result to " << filename << std::endl;
  plt::save(filename);
  plt::show();

  return 0;
}
