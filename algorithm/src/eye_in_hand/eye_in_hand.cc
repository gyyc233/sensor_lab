#include "eye_in_hand/eye_in_hand.h"
#include "transform/cv_eigen_convert.hpp"
#include "transform/eigenGeometryTransfer.hpp"
#include <iostream>

using namespace Algorithm;

EyeInHand::EyeInHand() {}

EyeInHand::~EyeInHand() {}

void EyeInHand::tsaiLenzHandEye(cv::Mat &Hcg, std::vector<cv::Mat> Hgij,
                                std::vector<cv::Mat> Hcij) {
  assert(Hgij.size() == Hcij.size());

  cv::Mat Rgij(3, 3, CV_64FC1); // rotation part of Hgij
  cv::Mat Rcij(3, 3, CV_64FC1); // rotation part of Hcij

  cv::Mat rgij(3, 1, CV_64FC1); // rotation vector of Rgij
  cv::Mat rcij(3, 1, CV_64FC1); // rotation vector of Rcij

  // rotation axis and normal vector
  double theta_gij;
  double theta_cij;
  cv::Mat rngij(3, 1, CV_64FC1);
  cv::Mat rncij(3, 1, CV_64FC1);

  cv::Mat Pgij(3, 1, CV_64FC1);
  cv::Mat Pcij(3, 1, CV_64FC1);

  // solve Ax=b
  cv::Mat tempA(3, 3, CV_64FC1);
  cv::Mat tempb(3, 1, CV_64FC1);
  cv::Mat A;
  cv::Mat b;
  cv::Mat A_invert;

  cv::Mat Pcg_prime(3, 1, CV_64FC1);
  cv::Mat Pcg(3, 1, CV_64FC1);
  cv::Mat PcgTranspose(1, 3, CV_64FC1);

  cv::Mat Rcg(3, 3, CV_64FC1); // 相机与末端的旋转关系
  cv::Mat eyeM = cv::Mat::eye(3, 3, CV_64FC1);

  for (int i = 0; i < Hgij.size(); i++) {
    Hgij[i](cv::Rect(0, 0, 3, 3)).copyTo(Rgij); // 提取旋转矩阵
    Hcij[i](cv::Rect(0, 0, 3, 3)).copyTo(Rcij);

    cv::Rodrigues(Rgij, rgij); // 旋转矩阵 转 旋转向量
    cv::Rodrigues(Rcij, rcij);

    theta_gij = cv::norm(rgij); // 范数
    theta_cij = cv::norm(rcij);
    rngij = rgij / theta_gij; // 归一化，轴角表示
    rncij = rcij / theta_cij;

    Pgij = 2 * sin(theta_gij / 2) * rngij; // 修正的罗德里格斯参数表示姿态变化
    Pcij = 2 * sin(theta_cij / 2) * rncij;

    // build Ax=b
    tempA = skew(Pgij + Pcij);
    tempb = Pcij - Pgij;
    A.push_back(tempA);
    b.push_back(tempb);
  }

  // 计算Ａ矩阵逆
  cv::invert(A, A_invert, cv::DECOMP_SVD);

  // estimate Rcg
  Pcg_prime = A_invert * b;
  Pcg = 2 * Pcg_prime / sqrt(1 + norm(Pcg_prime) * norm(Pcg_prime));
  PcgTranspose = Pcg.t();

  Rcg =
      (1 - norm(Pcg) * norm(Pcg) / 2) * eyeM +
      0.5 * (Pcg * PcgTranspose + sqrt(4 - norm(Pcg) * norm(Pcg)) * skew(Pcg));

  cv::Mat Tgij(3, 1, CV_64FC1);
  cv::Mat Tcij(3, 1, CV_64FC1);
  cv::Mat tempAA(3, 3, CV_64FC1);
  cv::Mat tempbb(3, 1, CV_64FC1);
  cv::Mat AA;
  cv::Mat bb;
  cv::Mat AA_invert;
  cv::Mat Tcg(3, 1, CV_64FC1); // 相机与末端的平移关系

  // 计算平移矩阵，也是Ax=b
  for (int i = 0; i < Hgij.size(); i++) {
    Hgij[i](cv::Rect(0, 0, 3, 3)).copyTo(Rgij);
    Hcij[i](cv::Rect(0, 0, 3, 3)).copyTo(Rcij);
    Hgij[i](cv::Rect(3, 0, 1, 3)).copyTo(Tgij);
    Hcij[i](cv::Rect(3, 0, 1, 3)).copyTo(Tcij);

    tempAA = Rgij - eyeM;
    tempbb = Rcg * Tcij - Tgij;

    AA.push_back(tempAA);
    bb.push_back(tempbb);
  }
  cv::invert(AA, AA_invert, cv::DECOMP_SVD);
  Tcg = AA_invert * bb;

  // 组装成齐次矩阵
  Rcg.copyTo(Hcg(cv::Rect(0, 0, 3, 3)));
  Tcg.copyTo(Hcg(cv::Rect(3, 0, 1, 3)));
  Hcg.at<double>(3, 0) = 0.0;
  Hcg.at<double>(3, 1) = 0.0;
  Hcg.at<double>(3, 2) = 0.0;
  Hcg.at<double>(3, 3) = 1.0;
}

cv::Mat EyeInHand::skew(const cv::Mat input_matrix) {
  assert(input_matrix.rows == 3 && input_matrix.cols == 1);

  cv::Mat skew_mat = cv::Mat::zeros(3, 3, CV_64FC1);
  skew_mat.at<double>(0, 1) = -input_matrix.at<double>(2, 0);
  skew_mat.at<double>(0, 2) = input_matrix.at<double>(1, 0);
  skew_mat.at<double>(1, 0) = input_matrix.at<double>(2, 0);
  skew_mat.at<double>(1, 2) = -input_matrix.at<double>(0, 0);
  skew_mat.at<double>(2, 0) = -input_matrix.at<double>(1, 0);
  skew_mat.at<double>(2, 1) = input_matrix.at<double>(0, 0);
  return skew_mat;
}

cv::Mat EyeInHand::dof6ZYXToTransformMatrix(double x, double y, double z,
                                            double euler_x, double euler_y,
                                            double euler_z) {
  Eigen::Matrix3d rotation_mat =
      euler2RotationMatrixSelfAxis(euler_x, euler_y, euler_z);
  cv::Mat rotation_Matrix =
      (cv::Mat_<double>(3, 3) << rotation_mat(0, 0), rotation_mat(0, 1),
       rotation_mat(0, 2), rotation_mat(1, 0), rotation_mat(1, 1),
       rotation_mat(1, 2), rotation_mat(2, 0), rotation_mat(2, 1),
       rotation_mat(2, 2));

  cv::Mat translation_matrix = (cv::Mat_<double>(3, 1) << x, y, z);

  cv::Mat transform_matrix(4, 4, CV_64FC1);
  rotation_Matrix.copyTo(transform_matrix(cv::Rect(0, 0, 3, 3)));
  translation_matrix.copyTo(transform_matrix(cv::Rect(3, 0, 1, 3)));

  transform_matrix.at<double>(3, 0) = 0.0;
  transform_matrix.at<double>(3, 1) = 0.0;
  transform_matrix.at<double>(3, 2) = 0.0;
  transform_matrix.at<double>(3, 3) = 1.0;
  return transform_matrix;
}

bool EyeInHand::transformMatrixToDof6ZYX(const cv::Mat &transform_matrix,
                                         double &x, double &y, double &z,
                                         double &rx_deg, double &ry_deg,
                                         double &rz_deg) {
  x = transform_matrix.at<double>(0, 3);
  y = transform_matrix.at<double>(1, 3);
  z = transform_matrix.at<double>(2, 3);
  rx_deg = atan2(transform_matrix.at<double>(2, 1),
                 transform_matrix.at<double>(2, 2)) *
           (180.0 / CV_PI);
  ry_deg = atan2(-transform_matrix.at<double>(2, 0),
                 sqrt(1 - pow(transform_matrix.at<double>(2, 0), 2))) *
           (180.0 / CV_PI);
  rz_deg = atan2(transform_matrix.at<double>(1, 0),
                 transform_matrix.at<double>(0, 0)) *
           (180.0 / CV_PI);
  std::cout << "rx_deg: " << rx_deg << " ,ry_deg: " << ry_deg
            << " ,rz_deg: " << rz_deg << std::endl;
  return true;
}

bool EyeInHand::readTxt(std::string file_path, int col,
                        std::vector<double> &data) {
  std::ifstream ifs; //创建流对象

  ifs.open(file_path, std::ios::in); //打开文件
  if (!ifs.is_open())                //判断文件是否打开
  {
    std::cout << "failed to open file";
    return false;
  }

  std::vector<std::string> item; // 用于存放文件中的每一行字符串数据
  std::string temp; // 把文件中的一行数据作为字符串放入容器中

  while (getline(ifs, temp)) {
    item.push_back(temp);
  }

  // 处理每一行的文本数据
  for (auto it = item.begin(); it != item.end(); it++) {
    std::istringstream istr(
        *it); //其作用是把字符串分解为单词(在此处就是把一行数据分为单个数据)
    std::string str;

    int count = 0; //统计一行数据中单个数据个数
    //获取文件中的第 1、2 列数据
    while (istr >> str) //以空格为界，把istringstream中数据取出放入到依次s中
    {
      //获取第1列数据
      if (count == col) {
        double r = atof(
            str.c_str()); //数据类型转换，将string类型转换成double,如果字符串不是由数字组成，则字符被转化为
                          // 0
        data.push_back(r);
      }
      count++;
    }
  }
  return true;
}

bool EyeInHand::errorCalculationEyeInHand(std::vector<cv::Mat> T_B2Gs,
                                          cv::Mat T_G2C,
                                          std::vector<cv::Mat> T_C2Os,
                                          EyeInHandErrorEstimation &error) {

  assert(T_B2Gs.size() == T_C2Os.size()); // 判断输入数据的集合大小是否相等

  for (int i = 0; i < T_B2Gs.size(); i++) {
    double rectify_x, rectify_y, rectify_z, rectify_rx, rectify_ry, rectify_rz;
    double measure_x, measure_y, measure_z, measure_rx, measure_ry, measure_rz;
    cv::Mat T_G2C_inv;
    cv::invert(T_G2C, T_G2C_inv);
    cv::Mat rectify_A = T_G2C * T_C2Os[i] * T_G2C_inv;
    transformMatrixToDof6ZYX(rectify_A, rectify_x, rectify_y, rectify_z,
                             rectify_rx, rectify_ry,
                             rectify_rz); // 校正数据 转欧拉角ZYX位姿数据
    transformMatrixToDof6ZYX(T_B2Gs[i], measure_x, measure_y, measure_z,
                             measure_rx, measure_ry,
                             measure_rz); // 测量数据 转欧拉角ZYX位姿数据
    error.X_error.push_back(measure_x - rectify_x); // 记录误差值
    error.Y_error.push_back(measure_y - rectify_x);
    error.Z_error.push_back(measure_z - rectify_x);
    error.RX_error.push_back(measure_rx - rectify_rx);
    error.RY_error.push_back(measure_ry - rectify_ry);
    error.RZ_error.push_back(measure_rz - rectify_rz);
  }

  error.X_error_SampleStdDeviation = rootMeanSquare(error.X_error);
  error.Y_error_SampleStdDeviation = rootMeanSquare(error.Y_error);
  error.Z_error_SampleStdDeviation = rootMeanSquare(error.Z_error);
  error.RX_error_SampleStdDeviation = rootMeanSquare(error.RX_error);
  error.RY_error_SampleStdDeviation = rootMeanSquare(error.RY_error);
  error.RZ_error_SampleStdDeviation = rootMeanSquare(error.RZ_error);

  return true;
}

double EyeInHand::sampleStdDeviation(std::vector<double> data) {
  double sum = 0;
  double average = 0;
  double temp_sum = 0;
  double sample_stdDeviation = 0;
  int n = (int)data.size();

  for (auto iterator = data.begin(); iterator != data.end(); iterator++) {
    sum += *iterator;
  }
  average = sum / n;

  for (auto iterator = data.begin(); iterator != data.end(); iterator++) {
    temp_sum += pow((*iterator - average), 2);
  }

  sample_stdDeviation = sqrt(temp_sum / (n - 1));
  return sample_stdDeviation;
}

double EyeInHand::rootMeanSquare(std::vector<double> data) {
  double sum = 0;
  double root_mean_square = 0;
  int n = (int)data.size();

  for (auto iterator = data.begin(); iterator != data.end(); iterator++) {
    sum += pow(*iterator, 2);
  }
  root_mean_square = sqrt(sum / n);
  return root_mean_square;
}
