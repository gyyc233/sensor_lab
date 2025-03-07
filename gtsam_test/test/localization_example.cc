// use Pose2 variables (x, y, theta) to represent the robot positions
#include <gtsam/geometry/Pose2.h>

// use simple integer Keys to refer to the robot poses
#include <gtsam/inference/Key.h>

// use a BetweenFactor to model odometry measurements.
#include <gtsam/slam/BetweenFactor.h>

// We add all facors to a Nonlinear Factor Graph, as our factors are nonlinear.
#include <gtsam/nonlinear/NonlinearFactorGraph.h>

// 需要对每个Value指定一个初始值(这个值通常是gnss给的观测值或者猜测值)
#include <gtsam/nonlinear/Values.h>

// use the standard Levenberg-Marquardt solver
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>

// Once the optimized values have been calculated, we can also calculate the
// marginal covariance of desired variables
#include <gtsam/nonlinear/Marginals.h>

// 由于标准 GPS 测量仅提供位置信息，而不提供方向信息,
// 它还将使用标准的高斯噪声模型。因此，我们将从 NoiseModelFactorN 派生
#include <gtsam/nonlinear/NonlinearFactor.h>

using namespace std;

class UnaryFactor : public gtsam::NoiseModelFactorN<gtsam::Pose2> {
  // The factor will hold a measurement consisting of an (X,Y) location
  // We could this with a Point2 but here we just use two doubles
  double mx_, my_;

public:
  /// shorthand for a smart pointer to a factor
  typedef boost::shared_ptr<UnaryFactor> shared_ptr;

  // The constructor requires the variable key, the (X, Y) measurement value,
  // and the noise model
  UnaryFactor(gtsam::Key j, double x, double y,
              const gtsam::SharedNoiseModel &model)
      : gtsam::NoiseModelFactorN<gtsam::Pose2>(model, j), mx_(x), my_(y) {}

  ~UnaryFactor() override {}

  // Using the NoiseModelFactorN base class there are two functions that must be
  // overridden. The first is the 'evaluateError' function. This function
  // implements the desired measurement function, returning a vector of errors
  // when evaluated at the provided variable value. It must also calculate the
  // Jacobians for this measurement function, if requested. 计算残差项
  gtsam::Vector evaluateError(
      const gtsam::Pose2 &q,
      boost::optional<gtsam::Matrix &> H = boost::none) const override {
    // The measurement function for a GPS-like measurement h(q) which predicts
    // the measurement (m) is h(q) = q, q = [qx qy qtheta] The error is then
    // simply calculated as E(q) = h(q) - m: error_x = q.x - mx error_y = q.y -
    // my Node's orientation reflects in the Jacobian, in tangent space this is
    // equal to the right-hand rule rotation matrix H =  [ cos(q.theta)
    // -sin(q.theta) 0 ]
    //      [ sin(q.theta)   cos(q.theta) 0 ]

    // q是SE(2)
    // 二维函数
    // H，它只返回机器人的位置，没有角度theta，与传统的pose2不同，所以它的jacobian长的和SE2的不一样
    // 这里H是计算jacobian矩阵，因为扰动为0
    // 在gtsam中　jacobian一般写作H
    const gtsam::Rot2 &R = q.rotation();
    if (H) {
      (*H) = (gtsam::Matrix(2, 3) << R.c(), -R.s(), 0.0, R.s(), R.c(), 0.0)
                 .finished();
    }
    return (gtsam::Vector(2) << q.x() - mx_, q.y() - my_).finished();
  }

  // The second is a 'clone' function that allows the factor to be copied. Under
  // most circumstances, the following code that employs the default copy
  // constructor should work fine.
  gtsam::NonlinearFactor::shared_ptr clone() const override {
    return boost::static_pointer_cast<gtsam::NonlinearFactor>(
        gtsam::NonlinearFactor::shared_ptr(new UnaryFactor(*this)));
  }

  // Additionally, we encourage you the use of unit testing your custom factors,
  // (as all GTSAM factors are), in which you would need an equals and print, to
  // satisfy the GTSAM_CONCEPT_TESTABLE_INST(T) defined in Testable.h, but these
  // are not needed below.
}; // UnaryFactor

int main(int argc, char **argv) {
  // 1. Create a factor graph container and add factors to it
  gtsam::NonlinearFactorGraph graph;

  // 2a. Add odometry factors
  // For simplicity, we will use the same noise model for each odometry factor
  auto odometryNoise =
      gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector3(0.2, 0.2, 0.1));
  // Create odometry (Between) factors between consecutive poses
  // 加入odom观测边
  // 假设这些是odom的观测，链接两个顶点，是二元边
  graph.emplace_shared<gtsam::BetweenFactor<gtsam::Pose2>>(
      1, 2, gtsam::Pose2(2.0, 0.0, 0.0), odometryNoise);
  graph.emplace_shared<gtsam::BetweenFactor<gtsam::Pose2>>(
      2, 3, gtsam::Pose2(2.0, 0.0, 0.0), odometryNoise);

  // 2b. Add "GPS-like" measurements
  // We will use our custom UnaryFactor for this.
  auto unaryNoise = gtsam::noiseModel::Diagonal::Sigmas(
      gtsam::Vector2(0.1, 0.1)); // 10cm std on x,y
  // 加入like-gnss 观测边
  graph.emplace_shared<UnaryFactor>(1, 0.0, 0.0, unaryNoise);
  graph.emplace_shared<UnaryFactor>(2, 2.0, 0.0, unaryNoise);
  graph.emplace_shared<UnaryFactor>(3, 4.0, 0.0, unaryNoise);
  graph.print("\nFactor Graph:\n"); // print

  // 3. Create the data structure to hold the initialEstimate estimate to the
  // solution For illustrative purposes, these have been deliberately set to
  // incorrect values

  // 创建一些有误差的顶点(可以假设是通过imu积分得到)
  gtsam::Values initialEstimate;
  initialEstimate.insert(1, gtsam::Pose2(0.5, 0.0, 0.2));
  initialEstimate.insert(2, gtsam::Pose2(1.3, 0.1, -0.2));
  initialEstimate.insert(3, gtsam::Pose2(2.1, 0.1, 0.1));
  initialEstimate.print("\nInitial Estimate:\n"); // print

  // 4. Optimize using Levenberg-Marquardt optimization. The optimizer
  // accepts an optional set of configuration parameters, controlling
  // things like convergence criteria, the type of linear system solver
  // to use, and the amount of information displayed during optimization.
  // Here we will use the default set of parameters.  See the
  // documentation for the full set of parameters.
  gtsam::LevenbergMarquardtOptimizer optimizer(graph, initialEstimate);
  gtsam::Values result = optimizer.optimize();
  result.print("Final Result:\n");

  // 5. Calculate and print marginal covariances for all variables
  gtsam::Marginals marginals(graph, result);
  cout << "x1 covariance:\n" << marginals.marginalCovariance(1) << endl;
  cout << "x2 covariance:\n" << marginals.marginalCovariance(2) << endl;
  cout << "x3 covariance:\n" << marginals.marginalCovariance(3) << endl;

  // save factor graph as graphviz dot file
  // Render dot file using "dot -Tpng localization_example.dot -o
  // localization_example.png"
  graph.saveGraph("localization_example.dot", result);

  // Also print out to console
  graph.dot(cout, result);

  return 0;
}
