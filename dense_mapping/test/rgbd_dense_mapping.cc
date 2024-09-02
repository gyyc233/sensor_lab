#include <Eigen/Geometry>
#include <fstream>
#include <iostream>
#include <octomap/octomap.h> // for octomap
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

int main() {
  std::vector<cv::Mat> color_imgs, depth_imgs;
  std::vector<Eigen::Isometry3d, Eigen::aligned_allocator<Eigen::Isometry3d>>
      poses;

  std::ifstream fin("./data/rgbd/pose.txt");
  if (!fin) {
    std::cerr << "cannot find pose file" << std::endl;
    return 1;
  }

  for (int i = 1; i <= 5; i++) {
    color_imgs.push_back(
        cv::imread("./data/rgbd/color/" + std::to_string(i) + ".png"));
    depth_imgs.push_back(
        cv::imread("./data/rgbd/depth/" + std::to_string(i) + ".pgm"));

    double data[7] = {0};
    for (int i = 0; i < 7; i++) {
      fin >> data[i];
    }
    Eigen::Quaterniond q(data[6], data[3], data[4], data[5]);
    Eigen::Isometry3d T(q); // Eigen::Isometry3d 实质上为4*4矩阵
    T.pretranslate(Eigen::Vector3d(
        data[0], data[1],
        data[2])); // pretranslate, prerotate 基于世界坐标系平移,旋转
    poses.push_back(T);
  }

  // add point clouds
  double cx = 325.5;
  double cy = 253.5;
  double fx = 518.0;
  double fy = 519.0;
  double depthScale = 1000.0;

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud(
      new pcl::PointCloud<pcl::PointXYZRGB>);
  // octomap tree
  octomap::OcTree tree(0.01); // 参数为分辨率

  for (int i = 0; i < 5; i++) {
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr sub_cloud(
        new pcl::PointCloud<pcl::PointXYZRGB>);
    cv::Mat color = color_imgs[i];
    cv::Mat depth = depth_imgs[i];
    // cv::imshow("color", color);
    // cv::waitKey(0);
    Eigen::Isometry3d T = poses[i];
    // std::cout << "index: " << i << "pose: \n" << T.data() << std::endl;

    octomap::Pointcloud oct_cloud; // the point cloud in octomap

    // add point
    for (int v = 0; v < color.rows; v++) {
      for (int u = 0; u < color.cols; u++) {
        // get point depth
        unsigned int d = depth.ptr<unsigned short>(v)[u];
        if (d == 0)
          continue;
        if (d >= 7000)
          continue;

        // u,v to camera frame
        Eigen::Vector3d point;
        point[2] = double(d) / depthScale;
        point[0] = (u - cx) * point[2] / fx;
        point[1] = (v - cy) * point[2] / fy;

        // camera frame to world frame
        Eigen::Vector3d point_world = T * point;
        pcl::PointXYZRGB p;
        p.x = point_world[0];
        p.y = point_world[1];
        p.z = point_world[2];
        p.b = color.data[v * color.step + u * color.channels()];
        p.g = color.data[v * color.step + u * color.channels() + 1];
        p.r = color.data[v * color.step + u * color.channels() + 2];
        sub_cloud->points.push_back(p);
        oct_cloud.push_back(point_world[0], point_world[1], point_world[2]);
      }
    }
    *point_cloud += *sub_cloud;
    std::cout << "111" << std::endl;
    tree.insertPointCloud(oct_cloud,
                          octomap::point3d(T(0, 3), T(1, 3), T(2, 3)));
    std::cout << "222" << std::endl;
  }

  // voxel filter
  pcl::VoxelGrid<pcl::PointXYZRGB> voxel_filter;
  voxel_filter.setLeafSize(0.05, 0.05, 0.05); // resolution
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr tmp(
      new pcl::PointCloud<pcl::PointXYZRGB>);
  voxel_filter.setInputCloud(point_cloud);
  voxel_filter.filter(*tmp);
  tmp->swap(*point_cloud);

  if (point_cloud->size() > 0)
    pcl::io::savePCDFileBinary("map.pcd", *point_cloud);

  // 更新中间节点的占据信息并写入磁盘
  tree.updateInnerOccupancy();
  std::cout << "saving octomap ... " << std::endl;
  tree.writeBinary("octomap.bt");

  return 0;
}
