- [eigenvalue decomposition](#eigenvalue-decomposition)
- [Definition](#definition)


# eigenvalue decomposition

[eigenvalue and eigenvector see here](./eigenvalue_eigenvector.md) 该篇主要将特征值的计算，这里讲述特征值分解

# Definition

- 矩阵特征值分解是一种将矩阵分解为特征值和特征向量的方法，通常用于描述矩阵的线性变换性质
- 特征值分解是对于一个$n * n$矩阵Ａ，如果存在一个非零向量v和一个标量$\lambda$使得$Av = \lambda v$ 则称$\lambda$是A的特征值，v是A的特征向量
- 特征值分解的目标是将矩阵A分解为$A=Q\Lambda Q^{-1}$ 其中Q是由特征向量组成的矩阵，$\Lambda$是对角阵，对角线上元素是特征值

计算过程

![](./eigenvalue_decomposition/img_1.png)

参考文章

- [矩阵特征值分解的原理、计算与应用](https://blog.51cto.com/yingnanxuezi/12617095)
- [数值分析](https://o-o-sudo.github.io/numerical-methods/-eigen-decomposition.html)

