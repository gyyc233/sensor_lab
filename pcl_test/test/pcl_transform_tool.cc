#include "transform/eigenGeometryTransfer.hpp"
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <iostream>
#include <pcl/common/common_headers.h>
#include <pcl/common/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>

Eigen::Affine3f transform;
std::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
Eigen::Matrix3d rotation_mat;
Eigen::Vector4f offset_xyz;
Eigen::Vector4d fixed_axis_euler;

void point_pick_cb(const pcl::visualization::PointPickingEvent &event,
                   void *vis) {}

void area_Packing_cb(const pcl::visualization::AreaPickingEvent &event,
                     void *vis) {}

void keyboard_cb(const pcl::visualization::KeyboardEvent &event, void *vis) {

  std::shared_ptr<pcl::visualization::PCLVisualizer> viewer_tmp =
      *static_cast<std::shared_ptr<pcl::visualization::PCLVisualizer> *>(vis);

  if (event.keyDown()) {
    int key_sym = event.getKeyCode() - 'a';

    switch (key_sym) {
    case 'V' - 'a':
      offset_xyz[3] *= -1.0f;
      break;
    case 'F' - 'a':
      fixed_axis_euler[3] *= -1.0f;
      break;
    case 'Z' - 'a':
      offset_xyz[0] += 0.01f * offset_xyz[3];
      break;
    case 'X' - 'a':
      offset_xyz[1] += 0.01f * offset_xyz[3];
      break;
    case 'C' - 'a':
      offset_xyz[2] += 0.01f * offset_xyz[3];
      break;
    case 'A' - 'a':
      fixed_axis_euler[0] = 0.5 * fixed_axis_euler[3];
      fixed_axis_euler[1] = 0.0;
      fixed_axis_euler[2] = 0.0;
      break;
    case 'S' - 'a':
      fixed_axis_euler[0] = 0.0;
      fixed_axis_euler[1] = 0.5 * fixed_axis_euler[3];
      fixed_axis_euler[2] = 0.0;
      break;
    case 'D' - 'a':
      fixed_axis_euler[0] = 0.0;
      fixed_axis_euler[1] = 0.0;
      fixed_axis_euler[2] = 0.5 * fixed_axis_euler[3];
      break;
    default:
      break;
    }

    transform.translation() << offset_xyz[0], offset_xyz[1], offset_xyz[2];
    rotation_mat = euler2RotationMatrixSelfAxis(
        fixed_axis_euler[0], fixed_axis_euler[1], fixed_axis_euler[2]);
    transform.rotate(rotation_mat.cast<float>()); // add euler
    std::cout << "offset_xyz: " << offset_xyz.transpose() << std::endl;
    std::cout << "fixed_axis_euler: " << fixed_axis_euler.transpose()
              << std::endl;
    std::cout
        << "transform: "
        << rotationMatrix2euler(transform.rotation().cast<double>()).transpose()
        << std::endl;
    viewer_tmp->updateCoordinateSystemPose("target", transform);
  }
}

int main() {
  viewer = std::make_shared<pcl::visualization::PCLVisualizer>("3D Viewer");
  viewer->setBackgroundColor(128, 128, 128);
  viewer->addCoordinateSystem(0.2, "world", 0);

  transform = Eigen::Affine3f::Identity();
  rotation_mat = Eigen::Matrix3d::Identity();
  offset_xyz = Eigen::Vector4f::Zero();
  offset_xyz[3] = 1.0f;
  fixed_axis_euler = Eigen::Vector4d::Zero();
  fixed_axis_euler[3] = 1.0;
  viewer->registerKeyboardCallback(keyboard_cb, (void *)&viewer);
  viewer->registerAreaPickingCallback(area_Packing_cb, (void *)&viewer);
  viewer->registerPointPickingCallback(point_pick_cb, (void *)&viewer);

  viewer->addCoordinateSystem(0.13, transform, "target");
  viewer->initCameraParameters();

  while (!viewer->wasStopped()) {
    viewer->spinOnce(100);
  }
  return 0;
}
