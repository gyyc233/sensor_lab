/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    SFMdata.h
 * @brief   Simple example for the structure-from-motion problems
 * @author  Duy-Nguyen Ta
 */

/**
 * A structure-from-motion example with landmarks, default function arguments
 * give
 *  - The landmarks form a 10 meter cube
 *  - The robot rotates around the landmarks, always facing towards the cube
 * Passing function argument allows to specificy an initial position, a pose
 * increment and step count.
 */

#pragma once

// As this is a full 3D problem, we will use Pose3 variables to represent the
// camera positions and Point3 variables (x, y, z) to represent the landmark
// coordinates. Camera observations of landmarks (i.e. pixel coordinates) will
// be stored as Point2 (x, y). We will also need a camera object to hold
// calibration information and perform projections.
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Pose3.h>

// We will also need a camera object to hold calibration information and perform
// projections.
#include <gtsam/geometry/Cal3_S2.h>
#include <gtsam/geometry/PinholeCamera.h>

/* ************************************************************************* */
std::vector<gtsam::Point3> createPoints() {

  // Create the set of ground-truth landmarks
  std::vector<gtsam::Point3> points;
  points.push_back(gtsam::Point3(10.0, 10.0, 10.0));
  points.push_back(gtsam::Point3(-10.0, 10.0, 10.0));
  points.push_back(gtsam::Point3(-10.0, -10.0, 10.0));
  points.push_back(gtsam::Point3(10.0, -10.0, 10.0));
  points.push_back(gtsam::Point3(10.0, 10.0, -10.0));
  points.push_back(gtsam::Point3(-10.0, 10.0, -10.0));
  points.push_back(gtsam::Point3(-10.0, -10.0, -10.0));
  points.push_back(gtsam::Point3(10.0, -10.0, -10.0));

  return points;
}

/* ************************************************************************* */
std::vector<gtsam::Pose3> createPoses(
    const gtsam::Pose3 &init = gtsam::Pose3(gtsam::Rot3::Ypr(M_PI / 2, 0,
                                                             -M_PI / 2),
                                            gtsam::Point3(30, 0, 0)),
    const gtsam::Pose3 &delta = gtsam::Pose3(
        gtsam::Rot3::Ypr(0, -M_PI / 4, 0),
        gtsam::Point3(sin(M_PI / 4) * 30, 0, 30 * (1 - sin(M_PI / 4)))),
    int steps = 8) {

  // Create the set of ground-truth poses
  // Default values give a circular trajectory, radius 30 at pi/4 intervals,
  // always facing the circle center 默认值给出一个圆形轨迹，半径为 30，间隔为
  // pi/4，始终面向圆心
  std::vector<gtsam::Pose3> poses;
  int i = 1;
  poses.push_back(init);
  for (; i < steps; ++i) {
    // compose 右乘相对变换的位姿
    poses.push_back(poses[i - 1].compose(delta));
  }

  return poses;
}
