#include "brute_force_nearest_neighbour.h"
// #include <execution>

namespace sad {
int bfnn_point(CloudPtr cloud, const Vec3f &point) {
  auto min_it = std::min_element(
      cloud->points.begin(), cloud->points.end(),
      [&point](const PointType &pt1, const PointType &pt2) -> bool {
        return (pt1.getVector3fMap() - point).squaredNorm() <
               (pt2.getVector3fMap() - point).squaredNorm();
      });

  return std::distance(cloud->points.begin(), min_it);
}

std::vector<int> bfnn_point_k(CloudPtr cloud, const Vec3f &point, int k) {
  struct IndexAndDis {
    IndexAndDis() {}
    IndexAndDis(int index, double dis2) : index_(index), dis2_(dis2) {}
    int index_ = 0;
    double dis2_ = 0;
  };

  std::vector<IndexAndDis> index_and_dis(cloud->size());
  for (int i = 0; i < cloud->size(); ++i) {
    // record index and distance of each point
    index_and_dis[i] = {
        i, (cloud->points[i].getVector3fMap() - point).squaredNorm()};
  }

  std::sort(index_and_dis.begin(), index_and_dis.end(),
            [](const auto &d1, const auto &d2) { return d1.dis2_ < d2.dis2_; });

  std::vector<int> ret;
  // 将index_and_dis中前k个IndexAndDis中的index_赋值到ret
  std::transform(index_and_dis.begin(), index_and_dis.begin() + k,
                 std::back_inserter(ret),
                 [](const auto &d1) { return d1.index_; });
  return ret;
}

void bfnn_cloud(CloudPtr cloud1, CloudPtr cloud2,
                std::vector<std::pair<size_t, size_t>> &matches) {
  // 单线程版本
  std::vector<size_t> index(cloud2->size());
  // 带初始化的捕获列表　用捕获到的信息初始化一个新的变量，在函数体内使用这个新的变量
  // 同样，这个捕获方式也是有值拷贝捕获和引用捕获两种方式
  std::for_each(index.begin(), index.end(),
                [idx = 0](size_t &i) mutable { i = idx++; });

  matches.resize(index.size());
  // c++17 std::execution::seq(串行执行):
  // std::for_each(std::execution::seq, index.begin(), index.end(), [&](auto
  // idx) {
  //   matches[idx].second = idx;
  //   matches[idx].first = bfnn_point(cloud1, ToVec3f(cloud2->points[idx]));
  // });
  std::for_each(index.begin(), index.end(), [&](auto idx) {
    matches[idx].second = idx;
    matches[idx].first = bfnn_point(cloud1, ToVec3f(cloud2->points[idx]));
  });
}

void bfnn_cloud_mt(CloudPtr cloud1, CloudPtr cloud2,
                   std::vector<std::pair<size_t, size_t>> &matches) {
  // 先生成索引
  std::vector<size_t> index(cloud2->size());
  std::for_each(index.begin(), index.end(),
                [idx = 0](size_t &i) mutable { i = idx++; });

  // 并行化for_each
  // std::execution::par_unseq(并行且矢量化执行):
  matches.resize(index.size());
  // std::for_each(
  //     std::execution::par_unseq, index.begin(), index.end(), [&](auto idx) {
  //       matches[idx].second = idx;
  //       matches[idx].first = bfnn_point(cloud1,
  //       ToVec3f(cloud2->points[idx]));
  //     });
  std::for_each(index.begin(), index.end(), [&](auto idx) {
    matches[idx].second = idx;
    matches[idx].first = bfnn_point(cloud1, ToVec3f(cloud2->points[idx]));
  });
}

void bfnn_cloud_mt_k(CloudPtr cloud1, CloudPtr cloud2,
                     std::vector<std::pair<size_t, size_t>> &matches, int k) {
  // 先生成索引
  std::vector<size_t> index(cloud2->size());
  std::for_each(index.begin(), index.end(),
                [idx = 0](size_t &i) mutable { i = idx++; });

  // 并行化for_each
  matches.resize(index.size() * k);
  // std::for_each(
  //     std::execution::par_unseq, index.begin(), index.end(), [&](auto idx) {
  //       auto v = bfnn_point_k(cloud1, ToVec3f(cloud2->points[idx]), k);
  //       for (int i = 0; i < v.size(); ++i) {
  //         matches[idx * k + i].first = v[i];
  //         matches[idx * k + i].second = idx;
  //       }
  //     });
  std::for_each(index.begin(), index.end(), [&](auto idx) {
    auto v = bfnn_point_k(cloud1, ToVec3f(cloud2->points[idx]), k);
    for (int i = 0; i < v.size(); ++i) {
      matches[idx * k + i].first = v[i];
      matches[idx * k + i].second = idx;
    }
  });
}

} // namespace sad
