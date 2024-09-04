#ifndef _EYE_IN_HAND_H_
#define _EYE_IN_HAND_H_

#include "opencv2/opencv.hpp"
#include <Eigen/Core>
#include <Eigen/Dense>

namespace Algorithm {

struct EyeInHandErrorEstimation {
  // 误差记录
  std::vector<double> X_error;
  std::vector<double> Y_error;
  std::vector<double> Z_error;
  std::vector<double> RX_error;
  std::vector<double> RY_error;
  std::vector<double> RZ_error;

  // 样本标准差
  double X_error_SampleStdDeviation = 0;
  double Y_error_SampleStdDeviation = 0;
  double Z_error_SampleStdDeviation = 0;

  // 样本方差
  double RX_error_SampleStdDeviation = 0;
  double RY_error_SampleStdDeviation = 0;
  double RZ_error_SampleStdDeviation = 0;
};

class EyeInHand {
public:
  EyeInHand();
  ~EyeInHand();

  /// @brief 手眼标定函数（EyeInHand 眼在手上）
  /// @details Tsai-Lenz
  /// @param Hcg 相机坐标系与机械臂末端的变换
  /// @param Hgij 前后两次机械臂末端坐标系相对位置姿态的齐次变换矩阵集合
  /// @param Hcij 前后两次相机坐标系之间相对位置姿态的齐次变换矩阵集合
  /// @return
  void tsaiLenzHandEye(cv::Mat &Hcg, std::vector<cv::Mat> Hgij,
                       std::vector<cv::Mat> Hcij);

  /// @brief 反对称矩阵转换函数
  /// @details 计算反对称矩阵
  /// @param input_Matrix 输入 3*1 矩阵
  /// @return 反对称矩阵 3*3矩阵
  cv::Mat skew(const cv::Mat input_Matrix);

  /// @brief 6自由度位姿数据转齐次矩阵函数
  /// @details 欧拉角 ZYX 外旋顺序
  /// @param x 平移量X
  /// @param y 平移量Y
  /// @param z 平移量Z
  /// @param euler_x 欧拉角RX (deg)
  /// @param euler_y 欧拉角RY (deg)
  /// @param euler_z 欧拉角RZ (deg)
  /// @return
  cv::Mat dof6ZYXToTransformMatrix(double x, double y, double z, double euler_x,
                                   double euler_y, double euler_z);

  /// @brief 齐次矩阵转6自由度位姿数据函数
  /// @details 依据齐次变换矩阵计算欧拉角顺序ZYX的6自由度位姿数据
  /// @param TransformMatrix 齐次变换矩阵
  /// @param x 平移量X
  /// @param y 平移量Y
  /// @param z 平移量Z
  /// @param rx_deg 欧拉角RX（deg）
  /// @param ry_deg 欧拉角RY（deg）
  /// @param rz_deg 欧拉角RZ（deg）
  /// @return
  bool transformMatrixToDof6ZYX(const cv::Mat &transform_matrix, double &x,
                                double &y, double &z, double &rx_deg,
                                double &ry_deg, double &rz_deg);

  /// @brief 读取 txt 文件某列数据
  /// @details 文件内容以空格分隔列
  /// @param filepath 文件路径
  /// @param col 取的列索引，0开始
  /// @param data 输出数据
  /// @return
  bool readTxt(std::string filepath, int col, std::vector<double> &data);

  /// @brief 重投影矩阵误差计算
  /// @param T_B2Gs 基座To执行末端的齐次变换阵集合
  /// @param T_G2C 标定的执行末端到相机的齐次变换矩阵结果
  /// @param T_C2Os 相机To靶标的齐次变换阵集合
  /// @param error 存储误差的结构体
  /// @return
  bool errorCalculationEyeInHand(std::vector<cv::Mat> T_B2Gs, cv::Mat T_G2C,
                                 std::vector<cv::Mat> T_C2Os,
                                 EyeInHandErrorEstimation &error);

  /// @brief 样本标准差计算
  /// @details 用于评估数据的离散分布情况
  /// @param data 输入数据
  /// @return 标准差值
  double sampleStdDeviation(std::vector<double> data);

  /// @brief 均方根计算
  /// @details 计算有效值，一般用于分析噪声
  /// @param data 输入数据
  /// @return 均方根值
  double rootMeanSquare(std::vector<double> data);
};
} // namespace Algorithm

#endif
