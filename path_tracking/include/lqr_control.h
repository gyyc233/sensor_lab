#ifndef PATH_TRACKING_LQR_CONTROL
#define PATH_TRACKING_LQR_CONTROL

#define EPS 1.0e-4
#include <Eigen/Dense>
#include <iostream>
#include <vector>

class LQRControl {
public:
  LQRControl(int n);
  ~LQRControl();

  /// @brief 计算黎卡提方程
  /// @param A 机器人状态更新矩阵
  /// @param B 机器人控制更新矩阵
  /// @param Q
  /// 状态权重矩阵,半正定,通常取为对角阵；Q矩阵元素变大意味着希望跟踪偏差能够快速趋近于零
  /// @param R
  /// 控制权重矩阵,为正定的控制加权矩阵，R矩阵元素变大意味着希望控制输入能够尽可能小
  /// @return Ｐ
  Eigen::MatrixXd calRicatti(Eigen::MatrixXd &A, Eigen::MatrixXd &B,
                             Eigen::MatrixXd &Q, Eigen::MatrixXd &R);

  /// @brief LQR Control
  /// @param robot_state current robot pose [x,y,psi]
  /// @param refer_path reference path
  /// @param s0
  /// @param A state-error update matrix
  /// @param B input-error update matrix
  /// @param Q state weight
  /// @param R input weight
  /// @return　delta-ref_delta
  double lqrControl(std::vector<double> robot_state,
                    std::vector<std::vector<double>> refer_path, double s0,
                    Eigen::MatrixXd A, Eigen::MatrixXd B, Eigen::MatrixXd Q,
                    Eigen::MatrixXd R);

private:
  int n_; // iterator of estimate ricatti
};

#endif
