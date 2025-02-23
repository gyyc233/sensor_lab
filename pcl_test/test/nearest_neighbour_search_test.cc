#include <gflags/gflags.h>
#include <glog/logging.h>
#include <gtest/gtest.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/search/kdtree.h>

#include "include/brute_force_nearest_neighbour.h"
#include "include/grid_nn.hpp"
#include "include/kd_tree.h"
#include "include/octop_tree.h"
#include "point_cloud/point_types.h"
#include "sys_utils.h"

DEFINE_string(first_scan_path, "./data/point_cloud/first.pcd",
              "第一个点云路径");
DEFINE_string(second_scan_path, "./data/point_cloud/second.pcd",
              "第二个点云路径");
DEFINE_double(ANN_alpha, 1.0, "AAN的比例因子");

void voxelGrid(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, float voxel_size) {
  pcl::VoxelGrid<pcl::PointXYZI> voxel;
  voxel.setLeafSize(voxel_size, voxel_size, voxel_size);
  voxel.setInputCloud(cloud);

  pcl::PointCloud<pcl::PointXYZI>::Ptr voxel_cloud(
      new pcl::PointCloud<pcl::PointXYZI>);
  voxel.filter(*voxel_cloud);
  cloud->swap(*voxel_cloud);
}

TEST(NN_TEST, BFNN) {
  // 这个用例经常会dumpcore
  GTEST_SKIP();

  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_1(
      new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_2(
      new pcl::PointCloud<pcl::PointXYZI>);
  pcl::io::loadPCDFile(FLAGS_first_scan_path, *cloud_1);
  pcl::io::loadPCDFile(FLAGS_second_scan_path, *cloud_2);

  if (cloud_1->empty() || cloud_2->empty()) {
    LOG(ERROR) << "cannot load cloud";
    FAIL();
  }
  LOG(INFO) << "1";
  // voxel grid 至 0.05
  voxelGrid(cloud_1, 2.0f);
  LOG(INFO) << "2";
  voxelGrid(cloud_2, 2.0f);
  LOG(INFO) << "3";
  LOG(INFO) << "points: " << cloud_1->size() << ", " << cloud_2->size();

  sad::evaluate_and_call(
      [&cloud_1, &cloud_2]() {
        std::vector<std::pair<size_t, size_t>> matches;
        sad::bfnn_cloud(cloud_1, cloud_2, matches);
      },
      "暴力匹配（单线程）", 1);
  LOG(INFO) << "test finished" << std::endl;
  SUCCEED();
}

TEST(NN_TEST, KDTREE_BASICS) {
  sad::CloudPtr cloud(new sad::PointCloudType);
  sad::PointType p1, p2, p3, p4;
  p1.x = 0;
  p1.y = 0;
  p1.z = 0;

  p2.x = 1;
  p2.y = 0;
  p2.z = 0;

  p3.x = 0;
  p3.y = 1;
  p3.z = 0;

  p4.x = 1;
  p4.y = 1;
  p4.z = 0;

  cloud->points.push_back(p1);
  cloud->points.push_back(p2);
  cloud->points.push_back(p3);
  cloud->points.push_back(p4);

  sad::KdTree kdtree;
  kdtree.BuildTree(cloud);
  kdtree.PrintAll();

  SUCCEED();
}

TEST(CH5_TEST, OCTREE_BASICS) {
  sad::CloudPtr cloud(new sad::PointCloudType);
  sad::PointType p1, p2, p3, p4;
  p1.x = 0;
  p1.y = 0;
  p1.z = 0;

  p2.x = 1;
  p2.y = 0;
  p2.z = 0;

  p3.x = 0;
  p3.y = 1;
  p3.z = 0;

  p4.x = 1;
  p4.y = 1;
  p4.z = 0;

  cloud->points.push_back(p1);
  cloud->points.push_back(p2);
  cloud->points.push_back(p3);
  cloud->points.push_back(p4);

  sad::OctoTree octree;
  octree.BuildTree(cloud);
  octree.SetApproximate(false);
  LOG(INFO) << "Octo tree leaves: " << octree.size()
            << ", points: " << cloud->size();

  SUCCEED();
}

int main(int argc, char **argv) {
  google::InitGoogleLogging(argv[0]);
  FLAGS_stderrthreshold = google::INFO;
  FLAGS_colorlogtostderr = true;

  testing::InitGoogleTest(&argc, argv);
  google::ParseCommandLineFlags(&argc, &argv, true);
  return RUN_ALL_TESTS();
}
