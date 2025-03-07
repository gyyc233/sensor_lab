#include <gtsam/geometry/Pose2.h>
#include <gtsam/inference/Key.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/BetweenFactor.h>

/**
 * A simple 2D pose slam example
 *  - The robot moves in a 2 meter square
 *  - The robot moves 2 meters each step, turning 90 degrees after each step
 *  - The robot initially faces along the X axis (horizontal, to the right in
 * 2D)
 *  - We have full odometry between pose
 *  - We have a loop closure constraint when the robot returns to the first
 * position
 */

int main(int argc, char **argv) {
  // 1. Create a factor graph container and add factors to it
  gtsam::NonlinearFactorGraph graph;

  // 2a. assume the noise follow a gaussian distribution
  auto prior_noise =
      gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector3(0.3, 0.3, 0.1));
  // Add a prior on the first pose, setting it to the origin
  // 在第一个顶点添加先验并将其设置为原点 A prior factor consists of a mean and
  // a noise model (covariance matrix) 先验因子
  graph.addPrior(1, gtsam::Pose2(0, 0, 0), prior_noise);

  // For simplicity, we will use the same noise model for odometry and loop
  // closures
  auto model =
      gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector3(0.2, 0.2, 0.1));

  // 2b. Add odometry factors
  // Create odometry (Between) factors between consecutive poses
  graph.emplace_shared<gtsam::BetweenFactor<gtsam::Pose2>>(
      1, 2, gtsam::Pose2(2, 0, 0), model);
  graph.emplace_shared<gtsam::BetweenFactor<gtsam::Pose2>>(
      2, 3, gtsam::Pose2(2, 0, M_PI_2), model);
  graph.emplace_shared<gtsam::BetweenFactor<gtsam::Pose2>>(
      3, 4, gtsam::Pose2(2, 0, M_PI_2), model);
  graph.emplace_shared<gtsam::BetweenFactor<gtsam::Pose2>>(
      4, 5, gtsam::Pose2(2, 0, M_PI_2), model);

  // 2c. Add the loop closure constraint 添加回环约束
  // This factor encodes the fact that we have returned to the same pose. In
  // real systems, these constraints may be identified in many ways, such as
  // appearance-based techniques with camera images. We will use another Between
  // Factor to enforce this constraint:
  graph.emplace_shared<gtsam::BetweenFactor<gtsam::Pose2>>(
      5, 2, gtsam::Pose2(2, 0, M_PI_2), model);
  graph.print("\nFactor Graph:\n"); // print

  // 3. Create the data structure to hold the initialEstimate estimate to the
  // solution For illustrative purposes, these have been deliberately set to
  // incorrect values
  gtsam::Values initialEstimate;
  initialEstimate.insert(1, gtsam::Pose2(0.5, 0.0, 0.2));
  initialEstimate.insert(2, gtsam::Pose2(2.3, 0.1, -0.2));
  initialEstimate.insert(3, gtsam::Pose2(4.1, 0.1, M_PI_2));
  initialEstimate.insert(4, gtsam::Pose2(4.0, 2.0, M_PI));
  initialEstimate.insert(5, gtsam::Pose2(2.1, 2.1, -M_PI_2));
  initialEstimate.print("\nInitial Estimate:\n"); // print

  // 4. Optimize the initial values using a Gauss-Newton nonlinear optimizer
  // The optimizer accepts an optional set of configuration parameters,
  // controlling things like convergence criteria, the type of linear
  // system solver to use, and the amount of information displayed during
  // optimization. We will set a few parameters as a demonstration.
  gtsam::GaussNewtonParams parameters;
  // Stop iterating once the change in error between steps is less than this
  // value
  parameters.relativeErrorTol = 1e-5;
  // Do not perform more than N iteration steps
  parameters.maxIterations = 100;
  // Create the optimizer ...
  gtsam::GaussNewtonOptimizer optimizer(graph, initialEstimate, parameters);
  // ... and optimize
  gtsam::Values result = optimizer.optimize();
  result.print("Final Result:\n");

  // 5. Calculate and print marginal covariances for all variables
  // 计算每个顶点的协方差
  std::cout.precision(3);
  gtsam::Marginals marginals(graph, result);
  std::cout << "x1 covariance:\n"
            << marginals.marginalCovariance(1) << std::endl;
  std::cout << "x2 covariance:\n"
            << marginals.marginalCovariance(2) << std::endl;
  std::cout << "x3 covariance:\n"
            << marginals.marginalCovariance(3) << std::endl;
  std::cout << "x4 covariance:\n"
            << marginals.marginalCovariance(4) << std::endl;
  std::cout << "x5 covariance:\n"
            << marginals.marginalCovariance(5) << std::endl;

  graph.saveGraph("pose2_slam_example.dot", result);

  // Also print out to console
  graph.dot(std::cout, result);

  return 0;
}
