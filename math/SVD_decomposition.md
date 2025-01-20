- [奇异值分解 (Singular Value Decomposition，SVD)](#奇异值分解-singular-value-decompositionsvd)


# 奇异值分解 (Singular Value Decomposition，SVD)

奇异值分解 (Singular Value Decomposition，SVD) 是一种矩阵因子分解方法，是线性代数的概念。应用于数据降维、推荐系统和自然语言处理等领域，在机器学习中被广泛适用

设$A$为n阶方阵，若存在一个实数$\lambda$和一个非零的n*1向量x满足

$$
Ax = \lambda x \tag{1}
$$

则称$\lambda$是A的一个特征值，x是对应$\lambda$A的特征向量

计算出$A$的n个特征值$\lambda_1<=\lambda_2<=...<=\lambda_n$，以及这 n 个特征值所对应的特征向量${p_1,p_2, ...,p_n}$，如果这n个特征值线性无关，那么矩阵$A$就可以用下式的特征分解表示：$A=P\Lambda P^{-1}$

其中P是这n个特征向量所张成的$n\$

参考文章

- [SVD Decomposition](https://o-o-sudo.github.io/numerical-methods/cholesky-cholesky-decomposition.html)
