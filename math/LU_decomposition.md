
# LU分解

对于可逆方阵A, 可以将其分解为一个下三角矩阵L和上三角矩阵U的乘积（不可逆的方阵有些也可以进行LU分解），有时需要再乘上一个置换矩阵P

- LU分解可以被视为高斯消元法的矩阵形式。在数值计算上，LU分解经常被用来解线性方程组、且在求逆矩阵和计算行列式中都是一个关键的步骤

![](./LU_decomposition/img_1.png)

# LU分解与矩阵高斯消元法

- LU分解在本质上是高斯消元法的一种表达形式。实质上是将A通过初等行变换变成一个上三角矩阵，其变换矩阵就是一个单位下三角矩阵。杜尔里特算法（Doolittle algorithm）
- 从下至上地对矩阵A做初等行变换，将对角线左下方的元素变成零，然后再证明这些行变换的效果等同于左乘一系列单位下三角矩阵，这一系列单位下三角矩阵的乘积的逆就是L矩阵，它也是一个单位下三角矩阵。

下面介绍一下高斯消元法

# 高斯消元

高斯消元法：是为了求解线性方程组的。应用消元法求解的时候，通常会应用以下三种变换，并且每一种变换都不会改变方程组的解

- 交换方程组中任意两个方程的位置
- 用一个数乘某一个方程的左右两边
- 将一个方程的两边乘一个数然后加到另一个方程上


参考文章

- [高斯消元](https://www.cnblogs.com/horizonshd/p/15365988.html#algebra2.1)
- [待定系数](https://zhuanlan.zhihu.com/p/363948873)
- [矩阵的分解——LU分解](https://blog.csdn.net/qq_28972011/article/details/123935820?utm_medium=distribute.pc_relevant.none-task-blog-2~default~baidujs_baidulandingword~default-0-123935820-blog-138027566.235^v43^control&spm=1001.2101.3001.4242.1&utm_relevant_index=3)
