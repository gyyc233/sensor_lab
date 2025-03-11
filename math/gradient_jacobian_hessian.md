- [gradient, jacobian matrix, hessian matrix](#gradient-jacobian-matrix-hessian-matrix)
  - [gradient](#gradient)
  - [jacobian matrix](#jacobian-matrix)
  - [hessian matrix](#hessian-matrix)
  - [positive definite matrix 正定矩阵](#positive-definite-matrix-正定矩阵)
  - [hessian matrix and positive definite matrix](#hessian-matrix-and-positive-definite-matrix)
- [newton method](#newton-method)
  - [求平方根](#求平方根)
    - [牛顿法在优化中的推广](#牛顿法在优化中的推广)
      - [一维优化问题中的牛顿法](#一维优化问题中的牛顿法)
      - [多维优化问题中的牛顿法](#多维优化问题中的牛顿法)
      - [hessian matrix 为何要求正定](#hessian-matrix-为何要求正定)
      - [牛顿法在优化中的优缺点和注意事项](#牛顿法在优化中的优缺点和注意事项)
      - [牛顿法多元函数求极值](#牛顿法多元函数求极值)

# gradient, jacobian matrix, hessian matrix

## gradient

梯度向量定义: 目标函数f为单变量，是关于子变量向量$x=(x_1,x_2,...,x_n)^T$的函数，则单变量函数f对向量x求梯度g(x)，称为梯度向量

$$

g(x) = \nabla f(x) = (\frac{\partial f}{\partial x_1}, \frac{\partial f}{\partial x_2}, ..., \frac{\partial f}{\partial x_n})^T

$$

## jacobian matrix

在向量分析中, 雅可比矩阵是一阶偏导数以一定方式排列成的矩阵, 其行列式称为雅可比行列式

- 目标函数f为一个函数向量，$f=(f_1(x),f_2(x),...,f_m(x))^T$,自变量$x=(x_1,x_2,...,x_n)^T$, 函数向量f对x求梯度，结果为一个矩阵
- 行数为f的维数；列数为x的维度，称之为Jacobian矩阵, 其每一行都是由相应函数的梯度向量转置构成的；

$$

\begin{bmatrix}

\frac{\partial f_1}{\partial x_1}, \frac{\partial f_1}{\partial x_2}, ..., \frac{\partial f_1}{\partial x_n} \\

\frac{\partial f_2}{\partial x_1}, \frac{\partial f_2}{\partial x_2}, ..., \frac{\partial f_2}{\partial x_n} \\

..., ..., ... \\

\frac{\partial f_m}{\partial x_1}, \frac{\partial f_m}{\partial x_2}, ..., \frac{\partial f_m}{\partial x_n}

\end{bmatrix}

$$

jacobian 物理意义,是多元函数的导数

jacobian matrix J(p)是函数f在n维空间某点p处的导数，它是一个线性映射（因为它是一个矩阵，矩阵本身代表着线性变换），它代表着函数f在点p处的最优线性逼近，也就是当x足够靠近点p时我们有

$$
F(x) \approx　F(p) + J(p)*(x-p)
$$

如果雅可比矩阵只有一个元素，它就等于2维空间中曲线在某点处的导数

- 若m == n，那么jacobian matrix是一个方阵，于是可以取它的行列式，称为jacobian行列式determinant

## hessian matrix

Hessian矩阵是梯度向量`g(x)(n*1)`对自变量`x(1*n)`的Jacobian矩阵

$$

\begin{bmatrix}

\frac{\partial^2 f}{\partial x_1 \partial x_1}, \frac{\partial^2 f}{\partial x_1 \partial x_2}, ..., \frac{\partial^2 f}{\partial x_1 \partial x_n} \\

\frac{\partial^2 f}{\partial x_2 \partial x_1}, \frac{\partial^2 f}{\partial x_2 \partial x_2}, ..., \frac{\partial^2 f}{\partial x_2 \partial x_n} \\

..., ..., ... \\

\frac{\partial^2 f}{\partial x_n \partial x_1}, \frac{\partial^2 f}{\partial x_n \partial x_2}, ..., \frac{\partial^2 f}{\partial x_n \partial x_n}

\end{bmatrix}

$$


## positive definite matrix 正定矩阵

Positive（正数） ：在数学和统计学中，通常指大于零的数。在矩阵理论中，一个矩阵被称为正定，是因为它的性质类似于正数的性质

Definite（定） ：在这里，“definite” 指的是矩阵具有确定的、明确的性质。具体地说，正定矩阵是指对于所有非零向量x，都有 $x^T A x > 0$ 成立，其中A是正定矩阵


ex:

$$
A=\begin{pmatrix}
2, 1\\1,2
\end{pmatrix}
$$

$$
x=\begin{pmatrix}
1\\1
\end{pmatrix}
$$

计算 $x^T A x$

$$
x^T=(1,1)

x^T A=\begin{pmatrix}
1, 1
\end{pmatrix}

\begin{pmatrix}
2, 1\\1,2
\end{pmatrix} = (3,3) \\

x^T A x = \begin{pmatrix}
3, 3
\end{pmatrix}

\begin{pmatrix}
1\\1
\end{pmatrix} = 6
$$

说明对于向量x，矩阵Ａ是正定的

## hessian matrix and positive definite matrix

最优化中应用当Hessan matrix正定时，在这一点取极小值；当Hessan matrix负定时，在这一点取极大值

# newton method

在科学计算、工程、经济和数据科学等领域中，我们经常需要求解方程或寻找某个函数的最优值

- 牛顿法（Newton's Method）是数值分析与优化中常用且高效的迭代方法之一。它利用函数的导数信息来快速逼近方程的解或函数的极值点
- 求方程根：给定一个函数 $f(x)$，我们想找到满足 $f(x)=0$ 的 $x$

牛顿法为这些问题提供了一种利用导数信息的快速迭代方法。当初始猜测点较好且函数具有较好性质时，牛顿法可以实现二次收敛，即误差在每次迭代时大致按平方速度减小，快速逼近目标解

然而，牛顿法并非毫无代价和限制：
- 对于求方程根问题，需要能计算导数 $f'(x)$。
- 对于多维优化问题，需要计算并存储 Hessian 矩阵（即二阶导数矩阵），这可能非常昂贵。
- Hessian 必须适当地满足一定的条件（如正定性），否则可能不收敛或朝错误方向迭代

牛顿法解一维方程: 设有一维函数 $f(x)$，想求解方程$\mathbf{x} = 0$ 如果函数不容易解析求根，我们考虑数值方法。牛顿法的出发点是利用函数在当前猜测点附近的线性近似来改进猜测值。

推导步骤

1. 选择一个初始猜测点 $x_0$，要求离真正的根 $x^*$ 不要太远（如果太远，收敛性不可保证）
2. 在当前点 $x_n$ 处，使用泰勒展开对 $f(x)$ 作一阶近似：$f(x)\approx f(x_n) + f^`(x_n)(x-x_n) = 0$ 当 $x_n$ 接近根 $x^*$ 时，这个近似会比较好
3. 为求新点 $x_{n+1}$ 更接近根，我们希望 $f(x_{n+1})=0$，用近似代入上式$0=f(x_{n+1}) + f^`(x_n)(x_{n+1}-x_n)$
4. 解出 $x_{n+1}$：$x_{n+1} = x_n - \frac{f(x_n)}{f^`(x_n)}$
5. 然后用$x_{n+1}$进行迭代

## 求平方根

设 $f(x)=x^2-2$，求解 $f(x)=0$ 对应的正根为 $\sqrt{2}$。

- 选定 $x_0=1$
- $f'(x)=2x$

应用牛顿法公式：

$$
x_1=x_0- \frac{{x_0}^2-2}{2x_0} =1-\frac{1-2}{2} = 1.5 继续迭代 \\ 
x_２=x_１- \frac{{x_1}^2-2}{2x_1} =1.5-\frac{2.25-2}{3} = 1.4167
$$

多次迭代后，$x_n$ 快速收敛到 $\sqrt{2}\approx1.4142$

### 牛顿法在优化中的推广

在优化中，我们常见的问题是最小化函数 $f(\mathbf{x})$。若 $f(\mathbf{x})$ 可微且存在极值点 $\mathbf{x}^*$，则此点满足梯度为零：

这样，求极值点的问题就转化为求解方程 $\nabla f(\mathbf{x}) = \mathbf{0}$。这与前面求一维方程根是类似的问题，只不过这里是向量方程

#### 一维优化问题中的牛顿法

先考虑一维优化，即最小化 $f(x)$。极值点满足 $f'(x)=0$。将牛顿法应用于 $f'(x)=0$ 的求解：

$$
x_{n+1} = x_n - \frac{f(x_n)}{f^`(x_n)}
$$

这里，我们是将 $f'(x)$ 当作目标函数求根，而 $f''(x)$ 相当于该目标函数的导数。此时，如果 $f''(x_n)>0$，则说明在该点函数局部是“向上凸起”的，这样更新点可以更好地朝极小值方向前进。

#### 多维优化问题中的牛顿法

对于多维可微函数 $f:\mathbb{R}^n \to \mathbb{R}$，极值点满足 $\nabla f(\mathbf{x})=\mathbf{0}$。对 $\nabla f(\mathbf{x})$ 在 $\mathbf{x}_n$ 作一次泰勒展开（记 Hessian 矩阵为 $H(\mathbf{x})$）：

$$
\nabla f(x) \approx \nabla f(x_n) + H(x_n)(x-x_n)
$$

其中 $H(\mathbf{x}_n)$ 是 $f(\mathbf{x})$ 在点 $\mathbf{x}_n$ 处的 Hessian 矩阵，元素为二阶偏导 $\frac{\partial^2 f}{\partial x_i \partial x_j}$

当寻找 $\nabla f(\mathbf{x})=\mathbf{0}$ 的近似解时，有：

$$
0 = \nabla f(x_n) + H(x_n)(x_{n+_1}-x_n)
$$

求解得到迭代公式

$$
x_{n+1} = x_n - \frac{\nabla f(x_n)}{H(x_n)} \\

or \\

x_{n+1} = x_n - H(x_n)^{-1}\nabla f(x_n)

$$

这就是多维优化问题中的牛顿法

#### hessian matrix 为何要求正定

正定性是指对任意非零向量 $\mathbf{z}$，有 $\mathbf{z}^\top H(\mathbf{x}) \mathbf{z} > 0$。正定 Hessian 矩阵意味着在该点附近，函数是局部凸的

为什么要正定而不仅仅是可逆？原因是当我们想求最小值时，一个正定的 Hessian 保证：

1. 方向性正确

当 Hessian 正定，更新方向 $-H(\mathbf{x}_n)^{-1}\nabla f(\mathbf{x}_n)$ 是下降方向，也就是在这个方向上函数值会下降

2. 快速收敛

如果 Hessian 非正定，即使它可逆，也可能表示该点是鞍点或其他不理想的局部结构。这样，新点可能不是朝向最小值的方向，导致算法无法快速收敛，甚至可能朝错误方向移动

总结：可逆性使我们能求出方向，但正定性使方向成为真正的下降方向，从而保证最小化问题的牛顿法在极值点附近快速（通常为二次）收敛

#### 牛顿法在优化中的优缺点和注意事项

优点：
- 二次收敛速度：当接近极值点时，收敛非常快。
- 不需要手动选择步长（类似梯度下降中的学习率问题），步长由 Hessian 决定。

缺点与注意事项：

1. 计算成本高：在高维问题中，计算 Hessian 及其逆矩阵可能非常昂贵。
2. 对初始点敏感：如果初始点远离极值点，可能不收敛或收敛到局部非期望点。
3. Hessian 必须正定或至少在极值点附近表现为正定，否则需要修改策略（如拟牛顿法或加正则项）。

如果 Hessian 不可逆或非正定，人们常使用 拟牛顿法（Quasi-Newton Methods） 来近似 Hessian 或其逆，以降低计算难度并改善收敛行为。

#### 牛顿法多元函数求极值

设 $f(\mathbf{x})=x_1^2+x_2^2$，想求最小值点

- 梯度

$$
\nabla f(x) = \begin{bmatrix}
  2x_1\\2x_2
\end{bmatrix}
$$

- Hessian

$$
H(x) = \begin{bmatrix}
  2, 0\\0, 2
\end{bmatrix}
$$

$H(\mathbf{x})$ 是正定的（对任意非零向量都为正）。选择初始点 $\mathbf{x}_0=[1,1]^T$

$$
x_{1} = x_0 - H(x_0)^{-1}\nabla f(x_0) = \begin{bmatrix}
  1\\1
\end{bmatrix}

- 

\begin{bmatrix}
  1/2, 0 \\1/2, 0
\end{bmatrix}

\begin{bmatrix}
  2\\2
\end{bmatrix}

= \begin{bmatrix}
  1\\1
\end{bmatrix}

- \begin{bmatrix}
  1\\1
\end{bmatrix}

= \begin{bmatrix}
  0\\0
\end{bmatrix}
$$
最小点 $(0,0)$

参考文章

- [梯度vs Jacobian矩阵vs Hessian矩阵](https://www.cnblogs.com/cy0628/p/13815909.html)
- [牛顿法](https://zhuanlan.zhihu.com/p/13304929959)
