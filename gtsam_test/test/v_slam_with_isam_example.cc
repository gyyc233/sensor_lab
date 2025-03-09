/**
 * A structure-from-motion example with landmarks
 *  - The landmarks form a 10 meter cube 地图构成一个10mi的立方体
 *  - The robot rotates around the landmarks, always facing towards the cube
 * 机器人围绕地标旋转，始终面向立方体
 */

// For loading the data
#include "SFMdata.h"

// Camera observations of landmarks (i.e. pixel coordinates) will be stored as
// Point2 (x, y).
#include <gtsam/geometry/Point2.h>

// Each variable in the system (poses and landmarks) must be identified with a
// unique key. We can either use simple integer keys (1, 2, 3, ...) or symbols
// (X1, X2, L1). Here we will use Symbols
#include <gtsam/inference/Symbol.h>

// In GTSAM, measurement functions are represented as 'factors'. Several common
// factors have been provided with the library for solving robotics/SLAM/Bundle
// Adjustment problems. Here we will use Projection factors to model the
// camera's landmark observations. Also, we will initialize the robot at some
// location using a Prior factor.
#include <gtsam/slam/ProjectionFactor.h> // 使用投影因子来模拟相机的地标观测

// We want to use iSAM to solve the structure-from-motion problem incrementally,
// so include iSAM here
#include <gtsam/nonlinear/NonlinearISAM.h>

// iSAM requires as input a set of new factors to be added stored in a factor
// graph, and initial guesses for any new variables used in the added factors
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>

#include <vector>

int main(int argc, char *argv[]) {
  using namespace std;
  using namespace gtsam;

  // Define the camera calibration parameters
  Cal3_S2::shared_ptr K(new Cal3_S2(50.0, 50.0, 0.0, 50.0, 50.0));
  // Define the camera observation noise model
  auto noise = noiseModel::Isotropic::Sigma(2, 1.0); // one pixel in u and v

  // 创建一组真实地标边长为10的立方体顶点
  vector<Point3> points = createPoints();

  // 创建一组真实位姿
  vector<Pose3> poses = createPoses();

  // Create a NonlinearISAM object which will relinearize and reorder the
  // variables 创建一个 NonlinearISAM 对象，它将重新线性化并重新排序变量 every
  // "relinearizeInterval" updates
  int relinearize_interval = 3; // 重新排序之间的更新次数
  NonlinearISAM isam(relinearize_interval);

  // Create a Factor Graph and Values to hold the new data
  NonlinearFactorGraph graph;
  Values initialEstimate;

  // Loop over the different poses, adding the observations to iSAM
  // incrementally
  for (size_t i = 0; i < poses.size(); ++i) {
    // Add factors for each landmark observation
    for (size_t j = 0; j < points.size(); ++j) {
      // Create ground truth measurement 相机观测值
      PinholeCamera<Cal3_S2> camera(poses[i], *K);
      // 将真实路标投影到相机中(u,v)
      Point2 measurement = camera.project(points[j]);
      // Add measurement
      graph.emplace_shared<GenericProjectionFactor<Pose3, Point3, Cal3_S2>>(
          measurement, noise, Symbol('x', i), Symbol('l', j), K);
    }

    // 为当前位姿i设置初始值
    // Intentionally initialize the variables off from the ground truth
    Pose3 noise(Rot3::Rodrigues(-0.1, 0.2, 0.25), Point3(0.05, -0.10, 0.20));
    Pose3 initial_xi = poses[i].compose(noise);
    // Add an initial guess for the current pose
    initialEstimate.insert(Symbol('x', i), initial_xi);

    // If this is the first iteration, add a prior on the first pose to set the
    // coordinate frame and a prior on the first landmark to set the scale Also,
    // as iSAM solves incrementally, we must wait until each is observed at
    // least twice before adding it to iSAM.30cm std on x,y,z 0.1 ra

    // 第一次迭代，则在第一个姿势上添加先验以设置坐标框架，并在第一个地标上添加先验以设置比例
    // 由于 iSAM
    // 是逐步解决的，我们必须等到每个姿势至少被观察两次之后才能将其添加到 iSAM

    // 为首个路标和位姿添加先验信息，为所有路标点添加猜测值
    if (i == 0) {
      // Add a prior on pose x0, with 30cm std on x,y,z 0.1 rad on
      // roll,pitch,yaw (rotation noise, position noise)
      auto poseNoise = noiseModel::Diagonal::Sigmas(
          (Vector(6) << Vector3::Constant(0.1), Vector3::Constant(0.3))
              .finished());
      graph.addPrior(Symbol('x', 0), poses[0], poseNoise);

      // Add a prior on landmark l0
      auto pointNoise = noiseModel::Isotropic::Sigma(3, 0.1); // 各向同性噪声
      graph.addPrior(Symbol('l', 0), points[0], pointNoise);

      // Add initial guesses to all observed landmarks
      Point3 noise(-0.25, 0.20, 0.15);
      for (size_t j = 0; j < points.size(); ++j) {
        // Intentionally initialize the variables off from the ground truth
        Point3 initial_lj = points[j] + noise;
        initialEstimate.insert(Symbol('l', j), initial_lj);
      }
    } else {
      // Update iSAM with the new factors
      isam.update(graph, initialEstimate);
      Values currentEstimate = isam.estimate();
      cout << "****************************************************" << endl;
      cout << "Frame " << i << ": " << endl;
      currentEstimate.print("Current estimate: ");

      // Clear the factor graph and values for the next iteration
      graph.resize(0);
      initialEstimate.clear();
    }
  }
  return 0;
}
