- [Eigen Aliasing](#eigen-aliasing)
  - [Aliasing Example](#aliasing-example)
  - [解决别名问题](#解决别名问题)
  - [xxxInPlace()](#xxxinplace)
  - [Aliasing and matrix products](#aliasing-and-matrix-products)
  - [Summary](#summary)
  - [参考文章](#参考文章)


# Eigen Aliasing

在 Eigen 中，混叠是指在赋值语句中，同一个矩阵（或数组或向量）出现在赋值运算符的左侧和右侧。像 `mat = 2 * mat;` 或 `mat = mat.transpose();`这样的语句就表现出混叠。第一个示例中的混叠是无害的，但第二个示例中的混叠会导致意外结果

## Aliasing Example

```cpp
MatrixXi mat(3,3); 
mat << 1, 2, 3,   4, 5, 6,   7, 8, 9;
cout << "Here is the matrix mat:\n" << mat << endl;
 
// 此作业显示了混叠问题
mat.bottomRightCorner(2,2) = mat.topLeftCorner(2,2);
cout << "After the assignment, mat = \n" << mat << endl;
```

output
```cpp
Here is the matrix mat:
1 2 3
4 5 6
7 8 9
After the assignment, mat = 
1 2 3
4 1 2
7 4 1
```

问题语句`mat.bottomRightCorner(2,2) = mat.topLeftCorner(2,2);`此赋值表现出混叠：系数 mat(1,1) 同时出现在赋值左侧的块 mat.bottomRightCorner(2,2) 和右侧的块 mat.topLeftCorner(2,2) 中。赋值后，右下角的 (2,2) 条目应具有赋值前的 mat(1,1) 的值，即 5。然而，输出显示 mat(2,2) 实际上是 1。问题在于 Eigen 对 mat.topLeftCorner(2,2) 使用了惰性求值（参见 Expression templates in Eigen ）。结果类似于

```
mat(1,1) = mat(0,0);
mat(1,2) = mat(0,1);
mat(2,1) = mat(1,0);
mat(2,2) = mat(1,1);
```

因此， mat(2,2) 被赋予了新值 mat(1,1) 而不是旧值。下一节将解释如何通过调用 eval() 来解决这个问题。

当尝试缩小矩阵时，混叠现象更自然地出现。例如，表达式 vec = vec.head(n) 和 mat = mat.block(i,j,r,c) 表现出混叠现象。

## 解决别名问题

如果您理解了混叠问题的原因，那么解决该问题的方法就显而易见了：Eigen 必须将右侧完全评估为临时矩阵/数组，然后将其分配给左侧。函数 eval() 正是这样做的

例如，这是上面第一个例子的更正版本：
```cpp
MatrixXi mat(3,3); 
mat << 1, 2, 3,   4, 5, 6,   7, 8, 9;
cout << "Here is the matrix mat:\n" << mat << endl;
 
// eval() 解决了别名问题
mat.bottomRightCorner(2,2) = mat.topLeftCorner(2,2).eval();
cout << "After the assignment, mat = \n" << mat << endl;
```

output
```
Here is the matrix mat:
1 2 3
4 5 6
7 8 9
After the assignment, mat = 
1 2 3
4 1 2
7 4 5
```

## xxxInPlace()

如果有 xxxInPlace() 函数可用，那么最好使用它，因为它可以更清楚地表明您正在做什么。这也可能允许 Eigen 更积极地进行优化。以下是提供的一些 xxxInPlace() 函数：

|Original function|In-place function|
|---|---|
|MatrixBase::adjoint()|	MatrixBase::adjointInPlace()|
|DenseBase::reverse()|DenseBase::reverseInPlace()|
|LDLT::solve()|LDLT::solveInPlace()|
|LLT::solve()|LLT::solveInPlace()|
|TriangularView::solve()|TriangularView::solveInPlace()|
|DenseBase::transpose()|DenseBase::transposeInPlace()|

在使用 vec = vec.head(n) 之类的表达式缩小矩阵或向量的特殊情况下，可以使用 conservativeResize()

## Aliasing and matrix products

Matrix 乘法是 Eigen 中唯一默认假设混叠的运算，前提是目标矩阵未调整大小。因此，如果 matA 是方阵，则语句 matA = matA * matA; 是安全的。Eigen 中的所有其他运算都假设没有混叠问题，因为结果被分配给不同的矩阵，或者因为它是逐个分量的运算。

```cpp
MatrixXf matA(2,2); 
matA << 2, 0,  0, 2;
matA = matA * matA;
cout << matA;
```

output
```
4 0
0 4
```

然而，这是有代价的。当执行表达式 matA = matA * matA 时，Eigen 会在临时矩阵中计算乘积，并在计算后将其分配给 matA 。这很好。但是当乘积被分配给不同的矩阵（例如 matB = matA * matA ）时，Eigen 也会这样做。在这种情况下，将乘积直接计算到 matB 中比先将其计算到临时矩阵中并将该矩阵复制到 matB 更有效。

用户可以使用 noalias() 函数指示没有混叠，如下所示： matB.noalias() = matA * matA 。这允许 Eigen 直接将矩阵乘积 matA * matA 计算为 matB 

```cpp
MatrixXf matA(2,2), matB(2,2); 
matA << 2, 0,  0, 2;
 
// Simple but not quite as efficient
matB = matA * matA;
cout << matB << endl << endl;
 
// More complicated but also more efficient
matB.noalias() = matA * matA;
cout << matB;
```

output

```
4 0
0 4

4 0
0 4
```

当然，当确实存在混叠时，您不应该使用 noalias() 。如果您这样做，那么您可能会得到错误的结果：

```cpp
MatrixXf matA(2,2); 
matA << 2, 0,  0, 2;
matA.noalias() = matA * matA;
cout << matA;
```

```
4 0
0 4
```

## Summary
当相同的矩阵或数组系数同时出现在赋值运算符的左侧和右侧时，就会发生别名。

- 对于系数计算来说，混叠是无害的；这包括标量乘法和矩阵或数组加法。
- 当您将两个矩阵相乘时，Eigen 会假设发生混叠。如果您知道没有混叠，那么您可以使用 noalias() 。
- 在所有其他情况下，Eigen 假设不存在混叠问题，因此如果确实发生混叠，则会给出错误的结果。为了防止这种情况，您必须使用 eval() 或 xxxInPlace() 函数之一。

## 参考文章

- [Lazy Evaluation and Aliasing](https://eigen.tuxfamily.org/dox/TopicLazyEvaluation.html)
- [Aliasing](https://runebook.dev/cn/docs/eigen3/group__topicaliasing)
