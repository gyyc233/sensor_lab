- [gradient, jacobian matrix, hessian matrix](#gradient-jacobian-matrix-hessian-matrix)
  - [gradient](#gradient)
  - [jacobian matrix](#jacobian-matrix)
  - [hessian matrix](#hessian-matrix)
  - [positive definite matrix 正定矩阵](#positive-definite-matrix-正定矩阵)
  - [hessian matrix and positive definite matrix](#hessian-matrix-and-positive-definite-matrix)
- [newton method](#newton-method)
  - [求平方根](#求平方根)
    - [牛顿法在优化中的推广](#牛顿法在优化中的推广)

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

jacobian　matrix J(p)是函数f在n维空间某点p处的导数，它是一个线性映射（因为它是一个矩阵，矩阵本身代表着线性变换），它代表着函数f在点p处的最优线性逼近，也就是当x足够靠近点p时我们有

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

一维优化问题中的牛顿法

先考虑一维优化，即最小化 $f(x)$。极值点满足 $f'(x)=0$。将牛顿法应用于 $f'(x)=0$ 的求解：

$$
x_{n+1} = x_n - \frac{f(x_n)}{f^`(x_n)}
$$

这里，我们是将 $f'(x)$ 当作目标函数求根，而 $f''(x)$ 相当于该目标函数的导数。此时，如果 $f''(x_n)>0$，则说明在该点函数局部是“向上凸起”的，这样更新点可以更好地朝极小值方向前进。

多维优化问题中的牛顿法

对于多维可微函数 $f:\mathbb{R}^n \to \mathbb{R}$，极值点满足 $\nabla f(\mathbf{x})=\mathbf{0}$。对 $\nabla f(\mathbf{x})$ 在 $\mathbf{x}_n$ 作一次泰勒展开（记 Hessian 矩阵为 $H(\mathbf{x})$）：

参考文章

- [梯度vs Jacobian矩阵vs Hessian矩阵](https://www.cnblogs.com/cy0628/p/13815909.html)
- [牛顿法](https://zhuanlan.zhihu.com/p/13304929959)
