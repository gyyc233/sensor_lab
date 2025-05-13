#ifndef VINS_ESTIMATOR_FACTOR_MARGINALIZATION_FACTOR
#define VINS_ESTIMATOR_FACTOR_MARGINALIZATION_FACTOR

#include <ceres/ceres.h>
#include <cstdlib>
#include <pthread.h>
#include <unordered_map>

#include "../utility/utility.h"
#include "tic_toc.h"

namespace sensor_lab {
// 用于实现部分边缘功能，主要用于在滑动窗口优化中将一些旧的状态变量“边缘化”出去
// 从而控制计算复杂度并保持状态估计的实时性

const int NUM_THREADS = 4;

// 残差块
struct ResidualBlockInfo {
  ResidualBlockInfo(ceres::CostFunction *_cost_function,
                    ceres::LossFunction *_loss_function,
                    std::vector<double *> _parameter_blocks,
                    std::vector<int> _drop_set)
      : cost_function(_cost_function), loss_function(_loss_function),
        parameter_blocks(_parameter_blocks), drop_set(_drop_set) {}

  void Evaluate();

  // _cost_function _loss_function：定义残差计算与其权重
  ceres::CostFunction *cost_function;
  ceres::LossFunction *loss_function;
  std::vector<double *> parameter_blocks; // 指向与该残差相关的参数数据的指针
  std::vector<int> drop_set; // 表示哪些参数要被边缘化（丢弃）

  double **raw_jacobians;
  std::vector<
      Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>
      jacobians;             // 在evaluate中计算的雅可比矩阵
  Eigen::VectorXd residuals; // 在evaluate中计算的残差

  // 将全局维度（如 SE(3) 的 7 维）转换为局部切空间维度（如 6 维）
  int localSize(int size) { return size == 7 ? 6 : size; }
};

// 用于多线程并行处理边缘化过程
struct ThreadsStruct {
  std::vector<ResidualBlockInfo *>
      sub_factors;   // 分配给当前线程处理的残差块子集
  Eigen::MatrixXd A; // 线性化后的雅可比矩阵
  Eigen::VectorXd b; // 线性化后的残差
  std::unordered_map<long, int> parameter_block_size; // global size
  std::unordered_map<long, int> parameter_block_idx;  // local size
};

// 管理所有残差块，并准备边缘化所需的数据结构
class MarginalizationInfo {
public:
  ~MarginalizationInfo();
  int localSize(int size) const;
  int globalSize(int size) const;
  void addResidualBlockInfo(
      ResidualBlockInfo *residual_block_info); // 添加一个新的残差块
  void preMarginalize();                       // 评估残差和雅可比矩阵
  void marginalize(); // 通过 Schur 补操作进行边缘化，把指定变量从系统中移除
  std::vector<double *>
  getParameterBlocks(std::unordered_map<long, double *>
                         &addr_shift); // 根据地址获取当前参数的值

  std::vector<ResidualBlockInfo *> factors; // 所有残差块列表
  int m, n;
  std::unordered_map<long, int> parameter_block_size; // global size
  int sum_block_size;
  std::unordered_map<long, int> parameter_block_idx; // local size
  std::unordered_map<long, double *> parameter_block_data;

  // 保留下的参数信息
  std::vector<int> keep_block_size; // global size
  std::vector<int> keep_block_idx;  // local size
  std::vector<double *> keep_block_data;

  // 用于优化的线性化矩阵
  Eigen::MatrixXd linearized_jacobians;
  Eigen::VectorXd linearized_residuals;
  const double eps = 1e-8;
};

// 将边缘化后的信息封装成 Ceres 兼容的代价函数
class MarginalizationFactor : public ceres::CostFunction {
public:
  MarginalizationFactor(MarginalizationInfo *_marginalization_info);

  // 使用已存储的边缘化信息计算残差和雅可比矩阵
  virtual bool Evaluate(double const *const *parameters, double *residuals,
                        double **jacobians) const;

  // 指向包含所有边缘化数据的 MarginalizationInfo 对象
  MarginalizationInfo *marginalization_info;
};
} // namespace sensor_lab

#endif
