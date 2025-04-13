#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <iostream>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "bal_problem.h"
#include "ceres/ceres.h"
#include "gflags/gflags.h"
#include "glog/logging.h"
#include "snavely_reprojection_error.h"

// 最小化重投影误差，获取最优的机器人位姿估计

// clang-format makes the gflags definitions too verbose
// clang-format off

DEFINE_string(input, "", "Input File name");
DEFINE_string(initial_ply, "./data/cere_test/output_source.ply", "Export the BAL file data as a PLY file.");

// solver option params
DEFINE_int32(num_iterations, 20, "Number of ceres solver max iterations.");
DEFINE_int32(num_threads, -1, "Number of threads. -1 = std::thread::hardware_concurrency.");
DEFINE_double(eta, 1e-2, "Default value for eta. Eta determines the "
              "accuracy of each linear solve of the truncated newton step. "
              "Changing this parameter can affect solve performance."); // 线性求解器中牛顿截断步长的精度
DEFINE_double(max_solver_time, 1e32, "Maximum solve time in seconds.");

DEFINE_string(final_ply, "./data/cere_test/output_target.ply", "Export the refined BAL file data as a PLY "
              "file.");
DEFINE_string(output_ref_txt, "./data/cere_test/output_ref.txt", "Export the BAL reference file data as a TXT "
              "file.");
DEFINE_string(output_txt, "./data/cere_test/output.txt", "Export the BAL file data as a TXT "
              "file.");
// clang-format on

void buildProblem(MyCeres::BALProblem *bal_problem, ceres::Problem *problem) {
  const int point_block_size = bal_problem->point_block_size();
  const int camera_block_size = bal_problem->camera_block_size();
  double *points = bal_problem->mutable_points();
  double *cameras = bal_problem->mutable_cameras();

  // Observations is 2*num_observations long array observations =
  // [u_1, u_2, ... , u_n], where each u_i is two dimensional, the x
  // and y positions of the observation.
  const double *observations =
      bal_problem->observations(); // 获取观测值x y 数据
  for (int i = 0; i < bal_problem->num_observations(); ++i) {
    // 1. set cost function
    ceres::CostFunction *cost_function;
    cost_function = MyCeres::SnavelyReprojectionError::Create(
        observations[2 * i + 0], observations[2 * i + 1]);

    // 2. set loss function
    ceres::LossFunction *loss_function = new ceres::HuberLoss(1.0);

    // 3. add residuals
    // 每组数据都是首地址
    double *camera =
        cameras + camera_block_size * bal_problem->camera_index()[i];
    double *point = points + point_block_size * bal_problem->point_index()[i];
    problem->AddResidualBlock(cost_function, loss_function, camera, point);

    // if(i<10){
    //   std::cout << "id: " << i << ", camera rotation [0]: " << camera[0]
    //             << ", [1]: " << camera[1] << ", [2]: " << camera[2]
    //             << "; point [0]: " << point[0] << ", point [0]: " << point[1]
    //             << ", point [2]: " << point[2] << std::endl;
    // }
  }
}

void SetMinimizerOptions(ceres::Solver::Options *options) {
  options->max_num_iterations = FLAGS_num_iterations;
  options->minimizer_progress_to_stdout = true;
  options->linear_solver_type = ceres::DENSE_SCHUR;

  if (CERES_GET_FLAG(FLAGS_num_threads) == -1) {
    // get number of concurrency threads supported by the implementation
    // 获取硬件支持并发数
    const int num_available_threads =
        static_cast<int>(std::thread::hardware_concurrency());
    if (num_available_threads > 0) {
      options->num_threads = num_available_threads;
    }
  } else {
    options->num_threads = CERES_GET_FLAG(FLAGS_num_threads);
  }
  CHECK_GE(options->num_threads, 1);

  // set 牛顿截断步长
  options->eta = CERES_GET_FLAG(FLAGS_eta);
  // 最大求解时间
  options->max_solver_time_in_seconds = CERES_GET_FLAG(FLAGS_max_solver_time);
}

void SetSolverOptionsFromFlags(MyCeres::BALProblem *bal_problem,
                               ceres::Solver::Options *options) {
  SetMinimizerOptions(options);
}

void solverProblem(const char *filename) {
  MyCeres::BALProblem bal_problem(filename, false);
  if (!FLAGS_initial_ply.empty()) {
    bal_problem.WriteToPLYFile(FLAGS_initial_ply);
  }

  if (!FLAGS_output_ref_txt.empty()) {
    bal_problem.WriteToFile(FLAGS_output_ref_txt.c_str());
  }

  ceres::Problem problem;
  buildProblem(&bal_problem, &problem);

  ceres::Solver::Options options;
  SetSolverOptionsFromFlags(&bal_problem, &options);
  options.gradient_tolerance = 1e-16;
  options.function_tolerance = 1e-16;
  options.parameter_tolerance = 1e-16;

  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);
  std::cout << summary.FullReport() << std::endl;

  if (!FLAGS_initial_ply.empty()) {
    bal_problem.WriteToPLYFile(FLAGS_final_ply, 255, 0, 0);
  }
  if (!FLAGS_output_txt.empty()) {
    bal_problem.WriteToFile(FLAGS_output_txt.c_str());
  }
}

int main(int argc, char **argv) {
  GFLAGS_NAMESPACE::ParseCommandLineFlags(&argc, &argv, true);
  google::InitGoogleLogging(argv[0]);
  if (FLAGS_input.empty()) {
    LOG(ERROR) << "Usage: bundle_adjuster --input=bal_problem";
    return 1;
  }

  solverProblem(FLAGS_input.c_str());

  return 0;
}
