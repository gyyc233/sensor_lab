# Sensor Lab <!-- omit in toc -->

A repository of implementations and notes that provide services for the math, computer vision, AI, robotics, autonomous driving related methods I have studied

:construction:

- [Getting Started](#getting-started)
  - [Prerequisites](#prerequisites)
- [Algorithm](#algorithm)
- [Feature Detect](#feature-detect)
- [Pose Estimation](#pose-estimation)
- [Kalman Filter](#kalman-filter)
- [imu\_and\_gnss](#imu_and_gnss)
- [Lidar\_2d](#lidar_2d)
- [Lidar\_3d](#lidar_3d)
- [pcl\_test](#pcl_test)
- [cere\_test](#cere_test)
- [g2o\_test](#g2o_test)
- [Path Plan](#path-plan)
- [Path Tracking](#path-tracking)
- [task](#task)
- [Usage](#usage)

## Getting Started

### Prerequisites

- Ubuntu 20.04
- C++14
- OpenCV 4.2.0
- Eigen 3.3.7
- PCL 1.10.0
- Sophus, commit SHA `a621ff` [local build](./task/local_build_sophus.md), support SO(3) and SE(3) operator
- glog
- gflags
- gtest build gtest and `sudo cp libgtest*.a /usr/local/lib`
- Ceres 1.12.0, deal with complicated non-linear optimization problems like Bundle Adjustment
- G2O, like Ceres
- DBoW3, loop closure
- gtsam 4.2.0, currently learning how to use it
- Pangolin, installed but rarely used
- Python 3.8.10. I use the features of `matplotlibcpp` and run some python scripts
- ROS noetic

## Algorithm

this module summarizes some methods for tradition image process, hand-eye calibration, curve fit

**image undistortion**

![](./support_files/image/algorithm/img1.png)

## Feature Detect

- based orb operator match

![orb_image_good_match](./support_files/image/feature_detect/orb_image_good_match.png)

## Pose Estimation

## Kalman Filter

## imu_and_gnss

eskf gins (imu+gnss+odom)

![imu_and_gnss](./support_files/image/imu_and_gnss/imu_and_gnss.png)

## Lidar_2d

lidar 2d mapping global map

![2d_mapping_global_map](./support_files/image/lidar_2d/2d_mapping_global_map.png)

## Lidar_3d

direct NDT LO mapping and without loop closure

![direct_ndt_lo](./support_files/image/lidar_3d/direct_ndt_lo.png)

## pcl_test

## cere_test

## g2o_test

## Path Plan

this module conclude some common tips and knowledge in global or local path plan. 

**Dijkstra global path plan**
![Dijkstra](./support_files/image/path_plan/dijkstra_demo.png)

**A star global path plan**

![A*](./support_files/image/path_plan/astar_demo.png)

**DWA local path plan**

![dwa](./support_files/image/path_plan/dwa_demo.png)

**RRT* sample based local path plan**

![RRT*](./support_files/image/path_plan/rrt_star_demo.png)

**Bezier Curve local path plan**

![bezier curve](./support_files/image/path_plan/bezier_curve.png)

**B spline curve local path plan**

![b spline curve](./support_files/image/path_plan/b_spline_demo.png)

**Dubins Curve path plan**

![dubins curve LSR](./support_files/image/path_plan/dubins_curve_1.png)

![dubins curve LSL](./support_files/image/path_plan/dubins_curve_2.png)

**Reeds-Shepp**

TODO: 

## Path Tracking

**PID Controller**

![PID Controller](./support_files/image/path_tracking/pid_demo.png)

## task

## Usage

```bash
mkdir build && cd build

# enable debug
cmake ..
make -j4
make install
```
