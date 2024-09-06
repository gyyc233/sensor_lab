
A Kalman filter that linearizes about the current mean and covariance is referred to as an extended Kalman filter or EKF.

In something akin to a Taylor series, we can linearize the estimation around the current estimate using the partial derivatives of the process and measurement functions to compute estimates even in the face of non-linear relationships.  在类似于泰勒级数的东西中，我们可以使用过程和测量函数的偏导数围绕当前估计值线性化估计值，即使在面对非线性关系时也能计算估计值。

## extended Kalman filter

卡尔曼滤波对于解决匀速或者匀加速度运动这种线性模型很适用，但实际问题中，基本不存在匀速或者匀加速这么简单的情况,这也就引出了扩展卡尔曼滤波EKF，用来解决非线性问题

EKF的核心就是将非线性问题转换为线性问题然后用KF求解，对于非线性化的$f(x_k)$$h(x_k)$，采用泰勒展开的方式线性化,然后其余步骤都与KF相同，带入计算即可

- 计算方差时,EKF的状态转移矩阵A和观测矩阵Ｈ都是状态信息的jacobian矩阵
- 在预测公式部分，Ａ(指代下F)是f的jacobian矩阵
- 在更新公式部分，Ｈ是h的jacobian矩阵

![](./img/ekf/img1.png)

![](./img/ekf/img2.png)

参考

[扩展卡尔曼滤波（EKF）理论讲解与实例（matlab、python和C++代码）](https://blog.csdn.net/O_MMMM_O/article/details/106078679)
