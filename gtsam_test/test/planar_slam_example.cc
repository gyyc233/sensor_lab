/**
 * Simple robotics example using odometry measurements and
 * bearing-range(方位-范围) (laser) measurements A simple 2D planar slam example
 * with landmarks
 *  - The robot and landmarks are on a 2 meter grid
 *  - Robot poses are facing along the X axis (horizontal, to the right in 2D)
 *  - The robot moves 2 meters each step
 *  - We have full odometry between poses
 *  - We have bearing and range information for measurements
 *  - Landmarks are 2 meters away from the robot trajectory
 * 因子图参考《hand on gtsam》 ch5.1 Figure 10
 */

// As this is a planar SLAM example, we will use Pose2 variables (x, y, theta)
// to represent the robot positions and Point2 variables (x, y) to represent the
// landmark coordinates.
#include <gtsam/geometry/Point2.h>
#include <gtsam/geometry/Pose2.h>

// Each variable in the system (poses and landmarks) must be identified with a
// unique key. We can either use simple integer keys (1, 2, 3, ...) or symbols
// (X1, X2, L1). Here we will use Symbols
#include <gtsam/inference/Symbol.h>

// In GTSAM, measurement functions are represented as 'factors'. Several common
// factors have been provided with the library for solving robotics/SLAM/Bundle
// Adjustment problems. Here we will use a RangeBearing factor for the
// range-bearing measurements to identified landmarks, and Between factors for
// the relative motion described by odometry measurements. Also, we will
// initialize the robot at the origin using a Prior factor.
#include <gtsam/sam/BearingRangeFactor.h> // RangeBearing factor 作为雷达测距边
#include <gtsam/slam/BetweenFactor.h> // 二元边描述机器人相对运动

// When the factors are created, we will add them to a Factor Graph. As the
// factors we are using are nonlinear factors, we will need a Nonlinear Factor
// Graph.
#include <gtsam/nonlinear/NonlinearFactorGraph.h> // 非线性因子对应非线性因子图

// Finally, once all of the factors have been added to our factor graph, we will
// want to solve/optimize to graph to find the best (Maximum A Posteriori
// 最大后验) set of variable values. GTSAM includes several nonlinear optimizers
// to perform this step. Here we will use the common Levenberg-Marquardt solver
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>

// Once the optimized values have been calculated, we can also calculate the
// marginal covariance of desired variables
#include <gtsam/nonlinear/Marginals.h>

// The nonlinear solvers within GTSAM are iterative solvers, meaning they
// linearize the nonlinear functions around an initial linearization point, then
// solve the linear system to update the linearization point. This happens
// repeatedly until the solver converges to a consistent set of variable values.
// This requires us to specify an initial guess for each variable, held in a
// Values container. 这要求我们为每个变量指定一个初始猜测，并保存在 Values
// 容器中
#include <gtsam/nonlinear/Values.h>

int main(int argc, char **argv) {
  using namespace gtsam;
  using namespace std;

  NonlinearFactorGraph graph;

  static Symbol x1('x', 1), x2('x', 2), x3('x', 3);
  static Symbol l1('l', 1), l2('l', 2);

  // add a prior on pose 1
  Pose2 prior(0.0, 0.0, 0.0);
  auto prior_noise = noiseModel::Diagonal::Sigmas(
      Vector3(0.3, 0.3, 0.1)); // 30cm std on x,y, 0.1 rad on theta
  graph.addPrior(x1, prior, prior_noise);

  // add two odometry factors
  Pose2 odometry(2.0, 0.0, 0.0);
  auto odometry_noise = noiseModel::Diagonal::Sigmas(
      Vector3(0.2, 0.2, 0.1)); // 20cm std on x,y, 0.1 rad on theta
  graph.emplace_shared<BetweenFactor<Pose2>>(x1, x2, odometry, odometry_noise);
  graph.emplace_shared<BetweenFactor<Pose2>>(x2, x3, odometry, odometry_noise);

  // Add Range-Bearing measurements to two different landmarks
  auto measurement_noise = noiseModel::Diagonal::Sigmas(
      Vector2(0.1, 0.2)); // 0.1 rad std on bearing, 20cm on range
  // create the measurement values - indices are (pose id, landmark id)
  // 《hand on gtsam》 ch5.1 Figure 10
  Rot2 bearing11 = Rot2::fromDegrees(45), bearing21 = Rot2::fromDegrees(90),
       bearing32 = Rot2::fromDegrees(90);
  double range11 = std::sqrt(4.0 + 4.0), range21 = 2.0, range32 = 2.0;

  // Add Bearing-Range factors
  graph.emplace_shared<BearingRangeFactor<Pose2, Point2>>(
      x1, l1, bearing11, range11, measurement_noise);
  graph.emplace_shared<BearingRangeFactor<Pose2, Point2>>(
      x2, l1, bearing21, range21, measurement_noise);
  graph.emplace_shared<BearingRangeFactor<Pose2, Point2>>(
      x3, l2, bearing32, range32, measurement_noise);
  // Print
  graph.print("Factor Graph:\n");

  // Create (deliberately inaccurate) initial estimate
  Values initialEstimate;
  initialEstimate.insert(x1, Pose2(0.5, 0.0, 0.2));
  initialEstimate.insert(x2, Pose2(2.3, 0.1, -0.2));
  initialEstimate.insert(x3, Pose2(4.1, 0.1, 0.1));
  initialEstimate.insert(l1, Point2(1.8, 2.1));
  initialEstimate.insert(l2, Point2(4.1, 1.8));
  // Print
  initialEstimate.print("Initial Estimate:\n");

  // Optimize using Levenberg-Marquardt optimization. The optimizer
  // accepts an optional set of configuration parameters, controlling
  // things like convergence criteria, the type of linear system solver
  // to use, and the amount of information displayed during optimization.
  // Here we will use the default set of parameters.  See the
  // documentation for the full set of parameters.
  LevenbergMarquardtOptimizer optimizer(graph, initialEstimate);
  Values result = optimizer.optimize();
  result.print("Final Result:\n");

  // Calculate and print marginal covariances for all variables
  Marginals marginals(graph, result);
  print(marginals.marginalCovariance(x1), "x1 covariance");
  print(marginals.marginalCovariance(x2), "x2 covariance");
  print(marginals.marginalCovariance(x3), "x3 covariance");
  print(marginals.marginalCovariance(l1), "l1 covariance");
  print(marginals.marginalCovariance(l2), "l2 covariance");

  graph.saveGraph("planar_slam_example.dot", result);

  graph.dot(std::cout, result);

  return 0;
}
