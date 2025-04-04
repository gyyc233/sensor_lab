#ifndef BRUTE_FORCE_NEAREST_NEIGHBOUR_H
#define BRUTE_FORCE_NEAREST_NEIGHBOUR_H

#include "eigen_type/eigen_types.h"
#include "point_cloud/point_types.h"

#include <thread>

namespace sad {

/**
 * Brute-force Nearest Neighbour
 * @param cloud 点云
 * @param point 待查找点
 * @return 找到的最近点索引
 */
int bfnn_point(CloudPtr cloud, const Vec3f &point);

/**
 * Brute-force Nearest Neighbour, k近邻
 * @param cloud 点云
 * @param point 待查找点
 * @param k 近邻数
 * @return 找到的最近点索引
 */
std::vector<int> bfnn_point_k(CloudPtr cloud, const Vec3f &point, int k = 5);

/**
 * 对点云进行BF最近邻
 * @param cloud1  目标点云
 * @param cloud2  被查找点云
 * @param matches 两个点云内的匹配关系
 * @return
 */
void bfnn_cloud(CloudPtr cloud1, CloudPtr cloud2,
                std::vector<std::pair<int, int>> &matches);

/**
 * 对点云进行BF最近邻 多线程版本
 * @param cloud1
 * @param cloud2
 * @param matches
 */
void bfnn_cloud_mt(CloudPtr cloud1, CloudPtr cloud2,
                   std::vector<std::pair<size_t, size_t>> &matches);

/**
 * 对点云进行BF最近邻 多线程版本，k近邻
 * @param cloud1
 * @param cloud2
 * @param matches
 */
void bfnn_cloud_mt_k(CloudPtr cloud1, CloudPtr cloud2,
                     std::vector<std::pair<size_t, size_t>> &matches,
                     int k = 5);
} // namespace sad
#endif
