# Latin Letters in Markdown

| No.  | Lowercase  | Uppercase  |  English  |              IPA              |
| :--: | :--------: | :--------: | :-------: | :---------------------------: |
| $1$  |  $\alpha$  |    $A$     |  $alpha$  |          **/'ælfə/**          |
| $2$  |  $\beta$   |    $B$     |  $beta$   |    **/'bi:tə/or/'beɪtə/**     |
| $3$  |  $\gamma$  |  $\Gamma$  |  $gamma$  |          **/'gæmə/**          |
| $4$  |  $\delta$  |  $\Delta$  |  $delta$  |         **/'deltə/**          |
| $5$  | $\epsilon$ |    $E$     | $epsilon$ |        **/'epsɪlɒn/**         |
| $6$  |  $\zeta$   |    $Z$     |  $zeta$   |         **/'zi:tə/**          |
| $7$  |   $\eta$   |    $H$     |   $eta$   |          **/'i:tə/**          |
| $8$  |  $\theta$  |  $\Theta$  |  $theta$  |         **/'θi:tə/**          |
| $9$  |  $\iota$   |    $I$     |  $iota$   |         **/aɪ'əʊtə/**         |
| $10$ |  $\kappa$  |    $K$     |  $kappa$  |          **/'kæpə/**          |
| $11$ | $\lambda$  | $\lambda$  | $lambda$  |         **/'læmdə/**          |
| $12$ |   $\mu$    |    $M$     |   $mu$    |          **/mju:/**           |
| $13$ |   $\nu$    |    $N$     |   $nu$    |          **/nju:/**           |
| $14$ |   $\xi$    |   $\Xi$    |   $xi$    |   **/ksi/or/'zaɪ/or/'ksaɪ/**  |
| $15$ | $\omicron$ |    $O$     | $omicron$ | **/əu'maikrən/or/'ɑmɪ,krɑn/** |
| $16$ |   $\pi$    |   $\Pi$    |   $pi$    |           **/paɪ/**           |
| $17$ |   $\rho$   |    $P$     |   $rho$   |           **/rəʊ/**           |
| $18$ |  $\sigma$  |  $\Sigma$  |  $sigma$  |         **/'sɪɡmə/**          |
| $19$ |   $\tau$   |    $T$     |   $tau$   |       **/tɔ:/or/taʊ/**        |
| $20$ | $\upsilon$ | $\Upsilon$ | $upsilon$ |  **/'ipsilon/or/'ʌpsilɒn/**   |
| $21$ |   $\phi$   |   $\Phi$   |   $phi$   |           **/faɪ/**           |
| $22$ |   $\chi$   |    $X$     |   $chi$   |           **/kaɪ/**           |
| $23$ |   $\psi$   |   $\Psi$   |   $psi$   |          **/psaɪ/**           |
| $24$ |  $\omega$  |  $\Omega$  |  $omega$  |   **/'əʊmɪɡə/or/oʊ'meɡə/**    |


# expressions

$$ 
\begin{aligned}
\frac{\partial u}{\partial t} + u \frac{\partial u}{\partial x} = - \frac{1}{\rho} \frac{\partial p}{\partial x} + \nu \frac{\partial^2 u}{\partial x^2} \\
\frac{\partial \rho}{\partial t} + \frac{\partial (\rho u)}{\partial x} = 0
\end{aligned} 
$$

- $x_1, x_2, ..., x_n$

- $a^{b^c}$ 或 $a^{bc}$

- $ {X}_{abc}x^{def} $
- 分式 $\frac{numerator}{denominator}$ $\frac{1}{2}$
- 开方 $\sqrt{x}, \sqrt[3]{x}, \sqrt[n]{x}$
- 求和求积 $\sum_{i=0}^{n} i^2$ $\prod_{i=1}^{n} i$
- 极限 $\lim_{x \to 0} \frac{\sin x}{x} = 1$
- 积分 
  - $\int_{a}^{b} x^2 dx$ 
  - $\int_{a}^{b} \frac{\partial f(x)}{\partial x} dx$
- 矩阵
  - $\begin{bmatrix} 1 & 2 \\ 3 & 4 \end{bmatrix}$
  - $\begin{pmatrix} 1 & 2 \\ 3 & 4 \end{pmatrix}$
  - $\begin{vmatrix} 1 & 2 \\ 3 & 4 \end{vmatrix}$
  - $\begin{Vmatrix} 1 & 2 \\ 3 & 4 \end{Vmatrix}$
  - $\left\{\begin{matrix}1 & 2 \\3 & 4 \end{matrix}\right\}$
- 分段函数
$$ f(x) =
\begin{cases}
0 & x\leq 0 \\
x & 0<x<1 \\
1 & x\ge
1 \end{cases} $$
- 比例 $a \propto b$ $c \sim d$
- 矢量 $\vec{a}, \vec{b}, \vec{c}$
- 无穷大 $\infty$ $lim_{x\to\infty}f(x)$
- \times, \pm, \div: 分别表示乘号、正负号、除号 $\pm (a \times b + c \div d) $
- \limits （$$）: 限制上下标的位置，一般用于求和、求积、积分等符号
  -  $\sum\limits_{i=1}^n a_i$

# 角标

|角标|latex|
|----|----|
|左上与右下|$^{w}p_{i}$|
|右上与左下|$_{w}p^{i}$|

# matrix

$$
\begin{bmatrix}
{a_{11}}&{a_{12}}&{\cdots}&{a_{1n}}\\
{a_{21}}&{a_{22}}&{\cdots}&{a_{2n}}\\
{\vdots}&{\vdots}&{\ddots}&{\vdots}\\
{a_{m1}}&{a_{m2}}&{\cdots}&{a_{mn}}\\
\end{bmatrix}
$$

- pmatrix：小括号边框
- bmatrix：中括号边框
- Bmatrix：大括号边框
- vmatrix：单竖线边框
- Vmatrix：双竖线边框

## equation

$$\begin{cases}
a_1x+b_1y+c_1z=d_1\\
a_2x+b_2y+c_2z=d_2\\
a_3x+b_3y+c_3z=d_3\\
\end{cases}
$$