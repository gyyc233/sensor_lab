#ifndef ROS_CATKIN
#define ROS_CATKIN

#include "parameters.h"

namespace sensor_lab
    [double INIT_DEPTH;   // initial depth for triangulation
     double MIN_PARALLAX; // minimum parallax for triangulation
                          // 最小视差，用于筛选关键帧
     double ACC_N, ACC_W; // 加计噪声与随机游走
     double GYR_N, GYR_W; // 陀螺噪声与随机游走

     // imu-camera extrinsic parameters rotation and translation
     std::vector<Eigen::Matrix3d> RIC; std::vector<Eigen::Vector3d> TIC;

     Eigen::Vector3d G{0.0, 0.0, 9.8};

     // 加速度计和陀螺仪零偏的阈值
     double BIAS_ACC_THRESHOLD; double BIAS_GYR_THRESHOLD;

     // 求解器的最大运行时间与迭代次数
     double SOLVER_TIME; int NUM_ITERATIONS;

     // 是否估计外参、时间偏移以及是否使用滚动快门模型
     int ESTIMATE_EXTRINSIC; int ESTIMATE_TD; int ROLLING_SHUTTER;

     std::string EX_CALIB_RESULT_PATH; std::string VINS_RESULT_PATH;
     std::string IMU_TOPIC;

     double ROW, COL; // 图像高度与宽度
     double TD, TR;   // 时间偏移与滚动快门模型的读出时间

     template <typename T> T readParam(ros::NodeHandle & n, std::string name) {
       T ans;
       if (n.getParam(name, ans)) {
         ROS_INFO_STREAM("Loaded " << name << ": " << ans);
       } else {
         ROS_ERROR_STREAM("Failed to load " << name);
         n.shutdown();
       }
       return ans;
     }

     void readParameters(ros::NodeHandle & n) {
       std::string config_file;
       config_file = readParam<std::string>(n, "config_file");
       cv::FileStorage fsSettings(config_file, cv::FileStorage::READ);
       if (!fsSettings.isOpened()) {
         std::cerr << "ERROR: Wrong path to settings" << std::endl;
       }

       fsSettings["imu_topic"] >> IMU_TOPIC;

       SOLVER_TIME = fsSettings["max_solver_time"];
       NUM_ITERATIONS = fsSettings["max_num_iterations"];
       MIN_PARALLAX = fsSettings["keyframe_parallax"];
       MIN_PARALLAX = MIN_PARALLAX / FOCAL_LENGTH;

       std::string OUTPUT_PATH;
       fsSettings["output_path"] >> OUTPUT_PATH;
       VINS_RESULT_PATH = OUTPUT_PATH + "/vins_result_no_loop.csv";
       std::cout << "result path " << VINS_RESULT_PATH << std::endl;

       // create folder if not exists
       FileSystemHelper::createDirectoryIfNotExists(OUTPUT_PATH.c_str());

       std::ofstream fout(VINS_RESULT_PATH, std::ios::out);
       fout.close();

       ACC_N = fsSettings["acc_n"];
       ACC_W = fsSettings["acc_w"];
       GYR_N = fsSettings["gyr_n"];
       GYR_W = fsSettings["gyr_w"];
       G.z() = fsSettings["g_norm"];
       ROW = fsSettings["image_height"];
       COL = fsSettings["image_width"];
       ROS_INFO("ROW: %f COL: %f ", ROW, COL);

       ESTIMATE_EXTRINSIC = fsSettings["estimate_extrinsic"];
       if (ESTIMATE_EXTRINSIC == 2) {
         ROS_WARN(
             "have no prior about extrinsic param, calibrate extrinsic param");
         RIC.push_back(Eigen::Matrix3d::Identity());
         TIC.push_back(Eigen::Vector3d::Zero());
         EX_CALIB_RESULT_PATH = OUTPUT_PATH + "/extrinsic_parameter.csv";

       } else {
         if (ESTIMATE_EXTRINSIC == 1) {
           ROS_WARN(" Optimize extrinsic param around initial guess!");
           EX_CALIB_RESULT_PATH = OUTPUT_PATH + "/extrinsic_parameter.csv";
         }
         if (ESTIMATE_EXTRINSIC == 0)
           ROS_WARN(" fix extrinsic param ");

         cv::Mat cv_R, cv_T;
         fsSettings["extrinsicRotation"] >> cv_R;
         fsSettings["extrinsicTranslation"] >> cv_T;
         Eigen::Matrix3d eigen_R;
         Eigen::Vector3d eigen_T;
         cv::cv2eigen(cv_R, eigen_R);
         cv::cv2eigen(cv_T, eigen_T);
         Eigen::Quaterniond Q(eigen_R);
         eigen_R = Q.normalized();
         RIC.push_back(eigen_R);
         TIC.push_back(eigen_T);
         ROS_INFO_STREAM("Extrinsic_R : " << std::endl << RIC[0]);
         ROS_INFO_STREAM("Extrinsic_T : " << std::endl << TIC[0].transpose());
       }

       INIT_DEPTH = 5.0;
       BIAS_ACC_THRESHOLD = 0.1;
       BIAS_GYR_THRESHOLD = 0.1;

       TD = fsSettings["td"];
       ESTIMATE_TD = fsSettings["estimate_td"];
       if (ESTIMATE_TD)
         ROS_INFO_STREAM(
             "Unsynchronized sensors, online estimate time offset, initial td: "
             << TD);
       else
         ROS_INFO_STREAM("Synchronized sensors, fix time offset: " << TD);

       ROLLING_SHUTTER = fsSettings["rolling_shutter"];
       if (ROLLING_SHUTTER) {
         TR = fsSettings["rolling_shutter_tr"];
         ROS_INFO_STREAM(
             "rolling shutter camera, read out time per line: " << TR);
       } else {
         TR = 0;
       }

       fsSettings.release();
     }]

#endif
