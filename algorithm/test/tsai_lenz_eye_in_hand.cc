#include "eye_in_hand/eye_in_hand.h"
#include <iostream>
#include <memory>
#include <opencv2/opencv.hpp>
#include <string>
#include <vector>

using namespace Algorithm;

int main() {
  std::shared_ptr<EyeInHand> operator_ptr = std::make_shared<EyeInHand>();
  // 机械臂正向运动学获得机械臂末端坐标
  std::vector<double> data_b2g_x;
  std::vector<double> data_b2g_y;
  std::vector<double> data_b2g_z;
  std::vector<double> data_b2g_rx;
  std::vector<double> data_b2g_ry;
  std::vector<double> data_b2g_rz;

  // 读取机械臂末端位姿数据
  operator_ptr->readTxt("./data/eye_in_hand/T_b2g.txt", 0, data_b2g_x);
  operator_ptr->readTxt("./data/eye_in_hand/T_b2g.txt", 1, data_b2g_y);
  operator_ptr->readTxt("./data/eye_in_hand/T_b2g.txt", 2, data_b2g_z);
  operator_ptr->readTxt("./data/eye_in_hand/T_b2g.txt", 3, data_b2g_rx);
  operator_ptr->readTxt("./data/eye_in_hand/T_b2g.txt", 4, data_b2g_ry);
  operator_ptr->readTxt("./data/eye_in_hand/T_b2g.txt", 5, data_b2g_rz);

  // 读取标定板在相机坐标系下位姿
  std::vector<double> data_c2o_x;
  std::vector<double> data_c2o_y;
  std::vector<double> data_c2o_z;
  std::vector<double> data_c2o_rx;
  std::vector<double> data_c2o_ry;
  std::vector<double> data_c2o_rz;
  operator_ptr->readTxt("./data/eye_in_hand/T_c2o.txt", 0, data_c2o_x);
  operator_ptr->readTxt("./data/eye_in_hand/T_c2o.txt", 1, data_c2o_y);
  operator_ptr->readTxt("./data/eye_in_hand/T_c2o.txt", 2, data_c2o_z);
  operator_ptr->readTxt("./data/eye_in_hand/T_c2o.txt", 3, data_c2o_rx);
  operator_ptr->readTxt("./data/eye_in_hand/T_c2o.txt", 4, data_c2o_ry);
  operator_ptr->readTxt("./data/eye_in_hand/T_c2o.txt", 5, data_c2o_rz);

  std::vector<cv::Mat> T_A;
  std::vector<cv::Mat> T_B;
  for (int i = 0; i < data_b2g_x.size(); i++) {
    cv::Mat T_b2g; // 机械臂末端
    cv::Mat T_c2o; //标定板在相机坐标系下位姿
    T_b2g = operator_ptr->dof6ZYXToTransformMatrix(
        data_b2g_x.at(i), data_b2g_y.at(i), data_b2g_z.at(i), data_b2g_rx.at(i),
        data_b2g_ry.at(i), data_b2g_rz.at(i));
    T_c2o = operator_ptr->dof6ZYXToTransformMatrix(
        data_c2o_x.at(i), data_c2o_y.at(i), data_c2o_z.at(i), data_c2o_rx.at(i),
        data_c2o_ry.at(i), data_c2o_rz.at(i));

    T_A.push_back(T_b2g);
    T_B.push_back(T_c2o);
  }

  // 生成AX = XB中的A,B
  std::vector<cv::Mat> A;
  std::vector<cv::Mat> B;
  for (int i = 1; i < T_A.size(); i++) {
    cv::Mat temp_invA;
    cv::Mat temp_invB;
    cv::Mat item_A;
    cv::Mat item_B;
    cv::invert(T_A.at(i), temp_invA);
    item_A = temp_invA * T_A.at(i - 1);
    A.push_back(item_A);
    cv::invert(T_B.at(i - 1), temp_invB);
    item_B = T_B.at(i) * temp_invB;
    B.push_back(item_B);
  }

  std::cout << "==================== A Matrix ===================="
            << std::endl;
  for (int i = 0; i < A.size(); i++) {
    std::cout << "index: " << i << "\n" << A.at(i) << std::endl;
  }
  std::cout << "==================== B Matrix ===================="
            << std::endl;
  for (int i = 0; i < B.size(); i++) {
    std::cout << "index: " << i << "\n" << B.at(i) << std::endl;
  }

  std::cout << "==================== X Result ===================="
            << std::endl;

  cv::Mat X_result(4, 4, CV_64FC1);
  operator_ptr->tsaiLenzHandEye(X_result, A, B);
  std::cout << X_result << std::endl;

  EyeInHandErrorEstimation error;
  if (operator_ptr->errorCalculationEyeInHand(A, X_result, B, error)) {
    std::cout << "Sample Standard Deviation X :"
              << error.X_error_SampleStdDeviation << std::endl;
    std::cout << "Sample Standard Deviation Y :"
              << error.Y_error_SampleStdDeviation << std::endl;
    std::cout << "Sample Standard Deviation Z :"
              << error.Z_error_SampleStdDeviation << std::endl;
    std::cout << "Sample Standard Deviation RX :"
              << error.RX_error_SampleStdDeviation << std::endl;
    std::cout << "Sample Standard Deviation RY :"
              << error.RY_error_SampleStdDeviation << std::endl;
    std::cout << "Sample Standard Deviation RZ :"
              << error.RZ_error_SampleStdDeviation << std::endl;
  }

  return 0;
}
