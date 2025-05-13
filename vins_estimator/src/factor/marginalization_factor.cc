#include "marginalization_factor.h"
#include <iostream>

namespace sensor_lab {
// 实现了边缘化因子图模块，主要功能是在滑动窗口优化中将旧的状态变量从系统中“边缘化”出去，从而减少计算量并维持系统的实时性

// 1. 初始化残差和雅可比结构
// 2. 调用 Ceres 接口计算原始残差和雅可比
// 3. 若有损失函数，应用鲁棒核函数对残差和雅可比进行加权
// 4. 提升优化稳定性，防止异常值影响
void ResidualBlockInfo::Evaluate() {
  // 设置残差向量大小为当前代价函数定义的残差维度
  residuals.resize(cost_function->num_residuals());

  // 获取每个参数块的维度
  std::vector<int> block_sizes = cost_function->parameter_block_sizes();
  // 分配指针数组
  raw_jacobians = new double *[block_sizes.size()];
  // 初始化 jacobians 向量以存储 Eigen 格式的雅可比矩阵
  jacobians.resize(block_sizes.size());

  // 为每个参数块分配jacobian矩阵空间
  for (int i = 0; i < static_cast<int>(block_sizes.size()); i++) {
    // jacobians[i] 每个参数块对应的jacobian矩阵
    // 行数=残差维数，列数=参数块的维度
    jacobians[i].resize(cost_function->num_residuals(), block_sizes[i]);
    // 将 Eigen 矩阵数据指针赋给 raw_jacobians，供 Ceres 调用底层函数使用
    raw_jacobians[i] = jacobians[i].data();
    // dim += block_sizes[i] == 7 ? 6 : block_sizes[i];
  }

  // 计算残差与未加权的jacobian
  cost_function->Evaluate(parameter_blocks.data(), residuals.data(),
                          raw_jacobians);

  // 若有损失函数则进行加权处理
  // 通过损失函数（如 Huber、Cauchy）可以降低大残差项对优化的影响
  if (loss_function) {
    double residual_scaling_, alpha_sq_norm_;
    double sq_norm, rho[3];

    // a. 计算残差平方范数和损失函数输出
    sq_norm = residuals.squaredNorm();
    loss_function->Evaluate(sq_norm, rho);
    // rho[0]: 损失函数值，rho[1]: 导数值，rho[2]: 二阶导数值（可能为 NULL）
    // printf("sq_norm: %f, rho[0]: %f, rho[1]: %f, rho[2]: %f\n", sq_norm,
    // rho[0], rho[1], rho[2]);

    // 根据损失函数的梯度调整残差和雅可比的权重，提升优化稳定性
    double sqrt_rho1_ = sqrt(rho[1]);

    // b. 计算缩放系数和 alpha
    if ((sq_norm == 0.0) || (rho[2] <= 0.0)) {
      residual_scaling_ = sqrt_rho1_;
      alpha_sq_norm_ = 0.0;
    } else {
      const double D = 1.0 + 2.0 * sq_norm * rho[2] / rho[1];
      const double alpha = 1.0 - sqrt(D);
      residual_scaling_ = sqrt_rho1_ / (1 - alpha);
      alpha_sq_norm_ = alpha / sq_norm;
    }

    // c. 对雅可比进行加权和修正，使其更符合鲁棒核函数的假设
    for (int i = 0; i < static_cast<int>(parameter_blocks.size()); i++) {
      jacobians[i] = sqrt_rho1_ * (jacobians[i] -
                                   alpha_sq_norm_ * residuals *
                                       (residuals.transpose() * jacobians[i]));
    }

    // d. 缩放残差，与jacobian保持一致
    residuals *= residual_scaling_;
  }
}

MarginalizationInfo::~MarginalizationInfo() {
  // ROS_WARN("release marginlizationinfo");

  // 释放优化过程中保存的参数块原始值
  for (auto it = parameter_block_data.begin(); it != parameter_block_data.end();
       ++it)
    delete[] it->second;

  for (int i = 0; i < (int)factors.size(); i++) {
    // 删除用于向 Ceres 提供雅可比矩阵指针
    delete[] factors[i]->raw_jacobians;
    // 删除与残差块关联的代价函数对象
    // 这些对象通常是在添加残差块时通过 new ceres::AutoDiffCostFunction 创建
    delete factors[i]->cost_function;
    // 删除残差块
    delete factors[i];
  }
}

/// @brief
/// 将一个残差块信息（ResidualBlockInfo）添加到当前的边缘化结构中，用于后续进行
/// Schur 补和变量边缘化
/// @param residual_block_info
void MarginalizationInfo::addResidualBlockInfo(
    ResidualBlockInfo *residual_block_info) {
  factors.emplace_back(
      residual_block_info); // factors 保存了所有参与边缘化的残差项

  // 获取该残差块涉及的所有参数块指针
  std::vector<double *> &parameter_blocks =
      residual_block_info->parameter_blocks;
  // 获取每个参数块的大小
  std::vector<int> parameter_block_sizes =
      residual_block_info->cost_function->parameter_block_sizes();

  for (int i = 0;
       i < static_cast<int>(residual_block_info->parameter_blocks.size());
       i++) {
    double *addr = parameter_blocks[i];
    int size = parameter_block_sizes[i]; // 记录每个参数块地址对应的实际大小
    // 将指针转为 long 类型作为 key，避免直接使用指针比较问题
    // reinterpret_cast
    // 运算符并不会改变括号中运算对象的值，而是对该对象从位模式上进行重新解释
    parameter_block_size[reinterpret_cast<long>(addr)] = size;
  }

  // 标记需要被边缘化的参数块
  for (int i = 0; i < static_cast<int>(residual_block_info->drop_set.size());
       i++) {
    double *addr = parameter_blocks[residual_block_info->drop_set[i]];
    parameter_block_idx[reinterpret_cast<long>(addr)] =
        0; // 标记需边缘化的参数块
  }
}

// 通常在滑动窗口边缘化流程中调用顺序如下
// 1. addResidualBlockInfo(...)：添加残差块信息
// 2. preMarginalize()：线性化并保存当前参数值
// 3. marginalize()：执行 Schur 补，完成边缘化
// 4. 构造 MarginalizationFactor 并加入下一轮优化问题中

/// @brief
/// 边缘化流程的预处理阶段，负责对所有残差块进行线性化，并保存当前参数值，为后续边缘化提供基础数据支撑
void MarginalizationInfo::preMarginalize() {
  for (auto it : factors) {
    it->Evaluate(); // 计算残差快残差与jacobian矩阵

    // 保存每个参数块的原始值
    std::vector<int> block_sizes = it->cost_function->parameter_block_sizes();
    for (int i = 0; i < static_cast<int>(block_sizes.size()); i++) {
      // 获取该残差块所涉及的所有参数块地址和大小
      long addr = reinterpret_cast<long>(it->parameter_blocks[i]);
      int size = block_sizes[i];
      if (parameter_block_data.find(addr) == parameter_block_data.end()) {
        double *data = new double[size];
        memcpy(data, it->parameter_blocks[i], sizeof(double) * size);
        // 保存参数块当前值，供后续边缘化使用
        parameter_block_data[addr] = data;
      }
    }
  }
}

int MarginalizationInfo::localSize(int size) const {
  return size == 7 ? 6 : size;
}

int MarginalizationInfo::globalSize(int size) const {
  return size == 6 ? 7 : size;
}

/// @brief 多线程并行构建 Hessian 矩阵 A 和梯度向量 b 的线程函数
/// @note 在多个线程中并行计算增量方程中的 Hessian 矩阵 $ A = J^T J $ 和梯度 $ b
/// = J^T r $，供后续 Schur 补使用，并行机制: 多线程分割残差块，加速构建过程
/// @param threadsstruct
/// @return
void *ThreadsConstructA(void *threadsstruct) {
  // p 是一个结构体指针，包含当前线程需要处理的残差块、Hessian 矩阵 A 和梯度向量
  // b
  ThreadsStruct *p = ((ThreadsStruct *)threadsstruct);

  for (auto it : p->sub_factors) {
    // 遍历该残差块涉及的所有参数块
    for (int i = 0; i < static_cast<int>(it->parameter_blocks.size()); i++) {
      // a. 获取参数块的索引和尺寸
      int idx_i = p->parameter_block_idx[reinterpret_cast<long>(
          it->parameter_blocks[i])];
      int size_i = p->parameter_block_size[reinterpret_cast<long>(
          it->parameter_blocks[i])];

      // 如果是 SE(3) 类型（7维），则转为切空间维度（6维）
      if (size_i == 7)
        size_i = 6;

      // b. 提取对应的雅可比矩阵,如对 SE(3) 取前 6 列
      Eigen::MatrixXd jacobian_i = it->jacobians[i].leftCols(size_i);

      // 构建 Hessian 矩阵 A（J^T * J）
      for (int j = i; j < static_cast<int>(it->parameter_blocks.size()); j++) {
        int idx_j = p->parameter_block_idx[reinterpret_cast<long>(
            it->parameter_blocks[j])];
        int size_j = p->parameter_block_size[reinterpret_cast<long>(
            it->parameter_blocks[j])];
        if (size_j == 7)
          size_j = 6;
        Eigen::MatrixXd jacobian_j = it->jacobians[j].leftCols(size_j);
        if (i == j)
          p->A.block(idx_i, idx_j, size_i, size_j) +=
              jacobian_i.transpose() * jacobian_j;
        else {
          p->A.block(idx_i, idx_j, size_i, size_j) +=
              jacobian_i.transpose() * jacobian_j;
          p->A.block(idx_j, idx_i, size_j, size_i) =
              p->A.block(idx_i, idx_j, size_i, size_j).transpose();
        }
      }

      // 构建梯度向量 b（J^T * r）
      p->b.segment(idx_i, size_i) += jacobian_i.transpose() * it->residuals;
    }
  }
  return threadsstruct;
}

/// @brief 通过 Schur
/// 补技术从系统中移除部分变量（如滑动窗口中的旧帧），同时保留它们对剩余变量的影响
void MarginalizationInfo::marginalize() {
  int pos = 0;
  for (auto &it : parameter_block_idx) {
    it.second = pos;
    pos += localSize(parameter_block_size[it.first]);
  }

  m = pos; // 被边缘化的变量总维度

  for (const auto &it : parameter_block_size) {
    if (parameter_block_idx.find(it.first) == parameter_block_idx.end()) {
      parameter_block_idx[it.first] = pos;
      pos += localSize(it.second);
    }
  }

  n = pos - m; // // n: 保留下来的变量维度

  TicToc t_summing;
  // 初始化 Hessian 和梯度，并启动多线程构建
  Eigen::MatrixXd A(pos, pos);
  Eigen::VectorXd b(pos);
  A.setZero();
  b.setZero();

  TicToc t_thread_summing;
  pthread_t tids[NUM_THREADS];
  ThreadsStruct threadsstruct[NUM_THREADS];
  int i = 0;
  for (auto it : factors) {
    threadsstruct[i].sub_factors.push_back(it);
    i++;
    i = i % NUM_THREADS;
  }

  // 将残差块按线程数分组
  for (int i = 0; i < NUM_THREADS; i++) {
    TicToc zero_matrix;
    threadsstruct[i].A = Eigen::MatrixXd::Zero(pos, pos);
    threadsstruct[i].b = Eigen::VectorXd::Zero(pos);
    threadsstruct[i].parameter_block_size = parameter_block_size;
    threadsstruct[i].parameter_block_idx = parameter_block_idx;
    int ret = pthread_create(&tids[i], NULL, ThreadsConstructA,
                             (void *)&(threadsstruct[i]));
    if (ret != 0) {
      ROS_WARN("pthread_create error");
      ROS_BREAK();
    }
  }

  // 启动多线程并行计算
  for (int i = NUM_THREADS - 1; i >= 0; i--) {
    pthread_join(tids[i], NULL);
    A += threadsstruct[i].A;
    b += threadsstruct[i].b;
  }

  // 执行schur补
  // 提取被边缘化变量对应的 Hessian 子块，并确保其对称性
  Eigen::MatrixXd Amm =
      0.5 * (A.block(0, 0, m, m) + A.block(0, 0, m, m).transpose());
  // 计算特征值分解，防止矩阵奇异
  Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> saes(Amm);

  if (saes.eigenvalues().minCoeff() >= -1e-4) {
    std::cout << "min eigenvalue: " saes.eigenvalues().minCoeff() << std::endl;
  }

  // 构建 Amm 的伪逆，仅保留大于阈值的特征值
  Eigen::MatrixXd Amm_inv =
      saes.eigenvectors() *
      Eigen::VectorXd((saes.eigenvalues().array() > eps)
                          .select(saes.eigenvalues().array().inverse(), 0))
          .asDiagonal() *
      saes.eigenvectors().transpose();
  printf("error1: %f\n",
         (Amm * Amm_inv - Eigen::MatrixXd::Identity(m, m)).sum());

  // Schur 补消去变量 m 个参数
  Eigen::VectorXd bmm = b.segment(0, m);
  Eigen::MatrixXd Amr = A.block(0, m, m, n);
  Eigen::MatrixXd Arm = A.block(m, 0, n, m);
  Eigen::MatrixXd Arr = A.block(m, m, n, n);
  Eigen::VectorXd brr = b.segment(m, n);
  A = Arr - Arm * Amm_inv * Amr;
  b = brr - Arm * Amm_inv * bmm;

  // 对新的 Hessian 进行特征值分解
  Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> saes2(A);

  // 忽略小特征值，增强数值稳定性
  Eigen::VectorXd S =
      Eigen::VectorXd((saes2.eigenvalues().array() > eps)
                          .select(saes2.eigenvalues().array(), 0));
  Eigen::VectorXd S_inv =
      Eigen::VectorXd((saes2.eigenvalues().array() > eps)
                          .select(saes2.eigenvalues().array().inverse(), 0));

  Eigen::VectorXd S_sqrt = S.cwiseSqrt();
  Eigen::VectorXd S_inv_sqrt = S_inv.cwiseSqrt();

  // 构造一个新的等效残差和雅可比，供后续 Ceres 使用
  linearized_jacobians = S_sqrt.asDiagonal() * saes2.eigenvectors().transpose();
  linearized_residuals =
      S_inv_sqrt.asDiagonal() * saes2.eigenvectors().transpose() * b;
  std::cout << A << std::endl << std::endl;
  std::cout << linearized_jacobians << std::endl;
  printf("error2: %f %f\n",
         (linearized_jacobians.transpose() * linearized_jacobians - A).sum(),
         (linearized_jacobians.transpose() * linearized_residuals - b).sum());
}

// 负责提取未被边缘化的参数块地址与元信息，用于构造下一轮优化所需的因子
std::vector<double *> MarginalizationInfo::getParameterBlocks(
    std::unordered_map<long, double *> &addr_shift) {
  std::vector<double *> keep_block_addr;
  keep_block_size.clear();
  keep_block_idx.clear();
  keep_block_data.clear();

  // parameter_block_idx 记录每个参数块地址对应的偏移量
  for (const auto &it : parameter_block_idx) {
    // 如果偏移量小于 m，表示该参数块已被边缘化；否则保留
    if (it.second >= m) {
      keep_block_size.push_back(parameter_block_size[it.first]); // 参数块维度
      keep_block_idx.push_back(
          parameter_block_idx[it.first]); // 参数块在整个变量空间中的偏移量
      keep_block_data.push_back(
          parameter_block_data
              [it.first]); // 保存的原始参数值（来自preMarginalize）
      keep_block_addr.push_back(
          addr_shift[it.first]); // 根据输入映射 addr_shift 获取新地址
    }
  }

  // 计算保留参数总维度
  sum_block_size = std::accumulate(std::begin(keep_block_size),
                                   std::end(keep_block_size), 0);

  return keep_block_addr;
}

// 将边缘化后的信息封装为一个 Ceres 兼容的因子
MarginalizationFactor::MarginalizationFactor(
    MarginalizationInfo *_marginalization_info)
    : marginalization_info(_marginalization_info) {
  int cnt = 0;
  for (auto it : marginalization_info->keep_block_size) {
    // 告知 Ceres 每个参数块的维度
    mutable_parameter_block_sizes()->push_back(it);
    cnt += it;
  }
  // 告知 Ceres 当前因子输出多少维残差
  set_num_residuals(marginalization_info->n);

  // 构造完成后，MarginalizationFactor 对象就可以作为普通因子加入到 Ceres
  // 优化问题 problem.AddResidualBlock(new
  // MarginalizationFacto(marginalization_info), NULL, keep_block_addr);
};

// 该函数根据当前参数与边缘化信息构造残差，并提供对应的雅可比矩阵，供 Ceres
// 进行非线性优化
bool MarginalizationFactor::Evaluate(double const *const *parameters,
                                     double *residuals,
                                     double **jacobians) const {
  printf("internal addr,%d, %d\n", (int)parameter_block_sizes().size(),
         num_residuals());
  for (int i = 0; i < static_cast<int>(keep_block_size.size()); i++) {
    printf("unsigned %x\n", reinterpret_cast<unsigned long>(parameters[i]));
    printf("signed %x\n", reinterpret_cast<long>(parameters[i]));
    printf("jacobian %x\n", reinterpret_cast<long>(jacobians));
    printf("residual %x\n", reinterpret_cast<long>(residuals));
  }

  int n = marginalization_info->n; // 当前因子输出的残差维度
  int m = marginalization_info->m; // 被边缘化的变量总维度

  Eigen::VectorXd dx(n); // 构造状态增量 dx 保存当前参数与先验值之间的差值

  // 遍历所有保留下来的参数块
  for (int i = 0;
       i < static_cast<int>(marginalization_info->keep_block_size.size());
       i++) {
    int size = marginalization_info->keep_block_size[i];
    int idx = marginalization_info->keep_block_idx[i] - m;
    Eigen::VectorXd x =
        Eigen::Map<const Eigen::VectorXd>(parameters[i], size); // 当前参数值
    Eigen::VectorXd x0 = Eigen::Map<const Eigen::VectorXd>(
        marginalization_info->keep_block_data[i],
        size); // 线性化点的参数值，来自 preMarginalize）

    // 计算dx
    if (size != 7) {
      // 普通参数块：直接使用向量差
      dx.segment(idx, size) = x - x0;
    } else {
      // SE3 参数块 当前参数与先验值之间的差
      dx.segment<3>(idx + 0) = x.head<3>() - x0.head<3>(); // 平移

      // 旋转
      dx.segment<3>(idx + 3) =
          2.0 * Utility::positify(
                    Eigen::Quaterniond(x0(6), x0(3), x0(4), x0(5)).inverse() *
                    Eigen::Quaterniond(x(6), x(3), x(4), x(5)))
                    .vec();

      if (!((Eigen::Quaterniond(x0(6), x0(3), x0(4), x0(5)).inverse() *
             Eigen::Quaterniond(x(6), x(3), x(4), x(5)))
                .w() >= 0)) {
        dx.segment<3>(idx + 3) =
            2.0 *
            -Utility::positify(
                 Eigen::Quaterniond(x0(6), x0(3), x0(4), x0(5)).inverse() *
                 Eigen::Quaterniond(x(6), x(3), x(4), x(5)))
                 .vec();
      }
    }
  }

  // 构造最终残差
  Eigen::Map<Eigen::VectorXd>(residuals, n) =
      marginalization_info->linearized_residuals +
      marginalization_info->linearized_jacobians * dx;

  // 构造jacobian
  if (jacobians) {

    for (int i = 0;
         i < static_cast<int>(marginalization_info->keep_block_size.size());
         i++) {
      if (jacobians[i]) {
        int size = marginalization_info->keep_block_size[i],
            local_size = marginalization_info->localSize(size);
        int idx = marginalization_info->keep_block_idx[i] - m;
        Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic,
                                 Eigen::RowMajor>>
            jacobian(jacobians[i], n, size);
        jacobian.setZero();
        jacobian.leftCols(local_size) =
            marginalization_info->linearized_jacobians.middleCols(idx,
                                                                  local_size);
      }
    }
  }
  return true;
}

} // namespace sensor_lab
