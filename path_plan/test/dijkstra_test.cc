#include <iostream>
#include <vector>

#define SIZE 110

int map[SIZE][SIZE]; //邻接矩阵存储 adjacency matrix
int cost[SIZE];      // cost[i]表示起点到i这个点的距离
int used[SIZE];      //节点是否被访问
int n;               // vertex size

/// @brief shortest path for directed graph
/// @param from start point
/// @param to end point
/// @param vertex_size vertex size
/// @return
int dijkstra(int from, int to, int vertex_size);

void printAdjacencyMatrix(int vertex_size, int map[][SIZE]);

void printCost(int vertex_size, int cost[]);

void printUsed(int vertex_size, int used[]);

int dijkstra(int from, int to, int vertex_size) {
  // 初始化 cost and used
  for (int i = 0; i < vertex_size; i++) {
    cost[i] = map[from][i];
    used[i] = 0;
  }

  // 起点被设置为标记状态
  used[from] = 1;
  std::cout << "check init" << std::endl;
  printCost(vertex_size, cost);
  printUsed(vertex_size, used);

  int min = 0;
  int u = 0; //记录离起点最小距离的坐标

  // 需要统计起点分别到其它n-1点的最小距离,需要n-1次
  for (int i = 0; i < vertex_size; i++) {

    //找到未标记过的，距离起点距离最近的点
    min = __INT32_MAX__;
    for (int j = 1; j < vertex_size; j++) {
      if (used[j] == 0 && cost[j] < min) {
        std::cout << "cost[j]: " << cost[j] << std::endl;
        min = cost[j];
        u = j; // 保存下标
      }
    }
    used[u] = 1;

    // update cost[]
    for (int v = 1; v < vertex_size; v++) {
      if (map[u][v] < __INT32_MAX__) {
        std::cout << "u: " << u << ", v: " << v << std::endl;
        if (cost[v] > cost[u] + map[u][v]) {
          cost[v] = cost[u] + map[u][v];
        }
      }
    }

    printCost(vertex_size, cost);
    printUsed(vertex_size, used);
  }

  return cost[to];
}

void printAdjacencyMatrix(int vertex_size, int map[][SIZE]) {
  for (int i = 0; i < vertex_size; ++i) {
    for (int j = 0; j < vertex_size; ++j) {
      std::cout << " " << map[i][j];
    }
    std::cout << std::endl;
  }
  std::cout << std::endl;
}

void printCost(int vertex_size, int cost[]) {
  std::cout << "loop cost[]: " << std::endl;
  for (int i = 0; i < vertex_size; ++i) {
    std::cout << " " << cost[i];
  }
  std::cout << std::endl;
}

void printUsed(int vertex_size, int used[]) {
  std::cout << "loop used[]: " << std::endl;
  for (int i = 0; i < vertex_size; ++i) {
    std::cout << " " << used[i];
  }
  std::cout << std::endl;
}

int main() {
  n = 6;

  // init adjacency matrix
  //设一开始起点到其它每个点都不可达
  for (int i = 0; i <= n; ++i) {
    for (int j = 0; j <= n; ++j) {
      map[i][j] = __INT32_MAX__;
    }
  }

  map[0][1] = 6;
  map[0][2] = 3;
  map[1][2] = 2;
  map[1][3] = 5;
  map[2][3] = 3;
  map[2][4] = 4;
  map[3][4] = 2;
  map[3][5] = 3;
  map[4][5] = 5;

  // 邻接矩阵的转置与自己相等
  int temp = __INT32_MAX__;
  for (int i = 0; i < n; ++i) {
    for (int j = 0; j < n; ++j) {
      if (i == j)
        map[i][j] = 0;

      if (map[i][j] == temp)
        map[i][j] = map[j][i];
    }
  }

  printAdjacencyMatrix(n, map);
  int ret = dijkstra(0, n - 1, n);
  std::cout << "shortest distance: " << ret << std::endl;
}
