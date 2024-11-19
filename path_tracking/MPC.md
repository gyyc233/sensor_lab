- [MPC model prediction control](#mpc-model-prediction-control)
  - [Motivation](#motivation)
  - [MPC \& 最优控制 \& PID](#mpc--最优控制--pid)
  - [MPC 整体流程](#mpc-整体流程)
  - [MPC 设计](#mpc-设计)
- [基于 MPC 的无人车轨迹跟踪](#基于-mpc-的无人车轨迹跟踪)
  - [二次规划(QP) 求解器](#二次规划qp-求解器)
  - [MPC 与 LQR](#mpc-与-lqr)
  - [参考资料](#参考资料)

# MPC model prediction control

- MPC 模型预测控制，实际上是用优化方法来求解控制问题，或者说用优化方法的给出控制器的动作
- 它利用一个已有的模型、系统当前的状态和未来的控制量，来预测系统未来的输出，然后与我们期望的系统输出做比较，得到一个损失函数

损失函数 = （未来输出（**模型**，未来控制量，**当前状态**）- **期望输出**）^2

使得损失函数取最小值(这里和 VSLAM 的非线性优化概念相似，在控制中，一般求的是二次型的优化问题，也叫二次规划)

- 上式中模型、当前状态、期望输出都是已知的，因此只有未来控制量一个自变量
- 采用二次规划的方法求解出某个未来控制量，使得损失函数最小，这个**未来控制量的第一个元素**就是当前控制周期的控制量

## Motivation

1. 模型
   1. 机理模型
   2. 基于数据的模型(ex 神经网络训练的的model)
2. 预测
   1. 用模型进行预测
3. 控制
   1. 根据预测做出决策

## MPC & 最优控制 & PID

**最优控制**

- 最优控制强调的是“最优”，一般最优控制需要在整个时间域上进行求优化
- 最优控制常用解法有 1变分法，2极大值原理，3动态规划
- 最优控制由于过于强调最优性，而暴露出两个问题
  - 对于非线性的 包含复杂约束的情况难以求解
  - 需要对系统的模型精确的知道

**MPC 与 最优控制**

MPC 仅去考虑未来几个时间步，一定程度上牺牲了最优性

**MPC 与 PID**

PID 缺点

- PID控制器不具有“前瞻性”，在计算公式中有当前err，上个周期的err，及err累积和，但没有未来的err
- PID属于无模型控制，仅通过err控制进行控制器设计

MPC 优点

- MPC 善于处理多输入多输出系统 MIMO
  - 对于MIMO系统，PID需要为每个子系统单独设计PID控制器，由于存在耦合对于较大的系统难以实现
  - MPC控制器可以较好控制MIMO系统
- MPC 可以处理约束：安全性约束，上下阈值
- MPC 有一定预测能力
  - 最优控制要求在整个时间优化
  - 采用了一个折中的策略，既不是像最优控制那样考虑这个时域，也不是完全的贪婪控制仅仅考虑当前，而是考虑未来的有限时间域
- **每一时间步都需要在线优化**

## MPC 整体流程

预测区间与控制区间

1. 建立系统状态方程，并将其做离散化。在k时刻，我们可以测量出系统的当前状态y(k),在通过计算得到的u(k),u(k+1),u(k+2)...u(k+j),得到未来系统估计值y(k+1),y(k+2)...y(k+j)
2. 将预测状态估计的部分称为预测区间（Predictive Horizon）,指的是一次优化后预测未来输出的时间步的个数
3. 将控制估计的部分称为控制区间（Control Horizon），在得到最优输入之后，我们只施加当前时刻的输入u(k)，即控制区间的第一位控制输入

<img width="729" alt="img1" src="https://github.com/user-attachments/assets/64ae3426-65aa-4321-8422-06f89fb3844c">

约束

1. 对于约束，一般分为Hard约束和Soft约束，Hard约束是不可违背必须遵守的，在设计时不建议将输入输出都给予Hard约束，因为这两部的约束有可能是有重叠的，导致优化器会产生不可行解
2. Hard约束不能违反，Soft约束可以；比如Hard约束是刹车踩的幅度；Soft约束是速度
3. 建议输出采用Soft约束，而输入的话建议输入和输入参数变化率二者之间不要同时为Hard约束，可以一个Hard一个Soft

MPC流程

<img width="735" alt="img2" src="https://github.com/user-attachments/assets/4a24b5c8-dce9-48cf-a760-ce3d9a3e558b">

## MPC 设计

当模型是线性的时候（非线性系统可以线性化,一阶泰勒展开），MPC的设计求解一般使用二次规划方法

<img width="742" alt="img3" src="https://github.com/user-attachments/assets/1f5e66d4-e08b-47b6-b983-3ca004727d5f">

<img width="505" alt="img4" src="https://github.com/user-attachments/assets/bae29668-d904-40d0-b8e6-937a1b32d89e">

<img width="740" alt="img5" src="https://github.com/user-attachments/assets/44220471-8203-420e-a818-7e2eccc88f85">

# 基于 MPC 的无人车轨迹跟踪

已知汽车的状态偏差和控制偏差模型

<img width="741" alt="img6" src="https://github.com/user-attachments/assets/7a56a4fb-0b55-415e-85fc-1bc8ad4ded68">

<img width="735" alt="img7" src="https://github.com/user-attachments/assets/1f8466d4-09d9-474f-8c9a-d5c4b7f64468">

##  二次规划(QP) 求解器

- NLopt
- OSQP 

## MPC 与 LQR

||MPC|LQR|
|---|---|---|
|研究对象|线性和非线性均可|线性系统|
|状态方程|1. 对非线性的状态方程进行线性化<br></br>2. 对线性方程进行离散化|对线性系统就行离散化|
|目标函数|MPC目标函数一个是累计和，本质都是对代价的累计，当系统对终端状态要求极为严格时，目标函数会加一项终端代价函数|LQR目标函数一个是积分，对代价的累计|
|求解方法|QP求解器，生成控制序列|变分法求解黎卡提方程，获取控制序列|
|工作时域|求解预测时间段Np内的控制序列 ，并在下个周期后进行滚动优化，，每次只需控制序列的第一个值作为控制的输入|只求解一次，每个周期下取对应的控制量即可，而不考虑之前周期下实际与规划的误差|

## 参考资料

- [MPC的简单实现](https://zhuanlan.zhihu.com/p/141871796)
- [模型预测控制(MPC)实现轨迹跟踪](https://blog.csdn.net/weixin_42301220/article/details/124566369)
