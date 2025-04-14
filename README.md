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

- Ubuntu 20.04 (virtual machine); ROS noetic
- C++14; OpenCV 4.2.0; Eigen 3.3.7; PCL 1.10.0
- Sophus, commit SHA `a621ff` [local build](./task/local_build_sophus.md), support SO(3) and SE(3) operator
- glog; gflags; gtest build gtest and `sudo cp libgtest*.a /usr/local/lib`
- Ceres 1.12.0, deal with complicated non-linear optimization problems like Bundle Adjustment
- G2O, like Ceres
- DBoW3, images loop closure
- gtsam 4.2.0, optimize library
- Pangolin, installed but rarely used
- Python 3.8.10. I use the features of `matplotlibcpp` and run some python scripts
- yaml-cpp use .yaml

## Algorithm

this module summarizes some methods for tradition image process, hand-eye calibration, curve fit etc.

**image undistortion**

![](./support_files/image/algorithm/img1.png)

## Feature Detect

- based orb operator match

![orb_image_good_match](./support_files/image/feature_detect/orb_image_good_match.png)

## Pose Estimation

## Kalman Filter

## imu_and_gnss

eskf gins (imu+gnss+odom)

in the image below, the left shows the ESKF and the right side uses IMU pre-integration


![imu_and_gnss](./support_files/image/imu_and_gnss/imu_and_gnss.png)


## Lidar_2d

lidar 2d mapping global map

![2d_mapping_global_map](./support_files/image/lidar_2d/2d_mapping_global_map.png)

## Lidar_3d

incremental NDT LO mapping and without loop closure

![incremental_ndt_lo](./support_files/image/lidar_3d/incremental_ndt_lo.png)

## pcl_test

## cere_test

the below image show the easy use of the ceres optimization library

- left image was source BAL data
- right image use ceres for BA optimize,reduce some noise(running in virtual machine, low performance)

![ceres_test](./support_files/image/ceres_test/ceres_1.png)


## g2o_test

## Path Plan

this module conclude some common tips and knowledge in global or local path plan. 

<table>
    <tr>
        <td ><center><img src="./support_files/image/path_plan/dijkstra_demo.png" >Dijkstra global path plan </center></td>
        <td ><center><img src="./support_files/image/path_plan/astar_demo.png"  >A star global path plan</center></td>
    </tr>
</table>

<table>
    <tr>
        <td ><center><img src="./support_files/image/path_plan/dwa_demo.png" >DWA local path plan </center></td>
        <td ><center><img src="./support_files/image/path_plan/rrt_star_demo.png"  >RRT* sample based local path plan</center></td>
    </tr>
</table>

**Bezier Curve local path plan and B spline curve local path plan**

<table>
    <tr>
        <td ><center><img src="./support_files/image/path_plan/bezier_curve.png" >Bezier Curve </center></td>
        <td ><center><img src="./support_files/image/path_plan/b_spline_demo.png"  >B spline curve </center></td>
    </tr>
</table>

**Dubins Curve path plan**

<table>
    <tr>
        <td ><center><img src="./support_files/image/path_plan/dubins_curve_1.png" >dubins curve LSR </center></td>
        <td ><center><img src="./support_files/image/path_plan/dubins_curve_2.png"  >dubins curve LSL</center></td>
    </tr>
</table>

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
