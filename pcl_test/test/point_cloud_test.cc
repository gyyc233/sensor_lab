#include <iostream>
#include <pcl/common/io.h>
#include <pcl/io/pcd_io.h>
// #include <pcl/visualization/cloud_viewer.h>

int main() {
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::io::loadPCDFile("./data/milk.pcd", *cloud);
  // pcl::visualization::CloudViewer viewer("cloud viewer");
  // viewer.showCloud(cloud);
  // while (!viewer.wasStopped()) {
  // }

  return 0;
}
