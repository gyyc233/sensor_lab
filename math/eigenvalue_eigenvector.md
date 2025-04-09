- [特征值和特征向量](#特征值和特征向量)
  - [finding all eigenvalues](#finding-all-eigenvalues)
  - [finding all eigenvectors](#finding-all-eigenvectors)
  - [矩阵的迹和行列式](#矩阵的迹和行列式)
  - [OpenCV 计算特征值和特征矩阵](#opencv-计算特征值和特征矩阵)
  - [Eigen 计算特征值和特征向量](#eigen-计算特征值和特征向量)

# 特征值和特征向量

前缀eigen起源于德语意思是 proper(这里应该是专属的意思)、characteristic(特征的)

Definition

A是n阶方阵，如果存在一个实数$\lambda$和一个非零的n*1向量x满足

$$
Ax = \lambda x \tag{1}
$$

则称$\lambda$是A的一个特征值，x是对应$\lambda$A的特征向量

consider

$$
A = \begin{bmatrix}
    1& -1\\2& 4
\end{bmatrix}
$$

可以验证

$$
\begin{bmatrix}
    1& -1\\2& 4
\end{bmatrix}

\begin{bmatrix}
    1\\-2
\end{bmatrix} = 3
\begin{bmatrix}
    1\\-2
\end{bmatrix}
$$

则3是A的一个特征值, $\begin{bmatrix}
    1\\-2
\end{bmatrix}$是对应3的A的一个特征向量

## finding all eigenvalues

1. 基于(1)元素全移动到左边 $(A-\lambda I)x = 0$, 令$B=A-\lambda I$ 有$Bx=0$

$$
(A-\lambda I)x = 0 \tag{2}
$$

2. 令B的行列式$det(B)=0$计算方程的根$\lambda$ 

以上述例子计算

$$
B = A - \lambda I\\
B = \begin{bmatrix}
    1-\lambda & -1\\
    2& 4-\lambda
\end{bmatrix} \\
$$

$$
det(B) = \lambda ^2 - 5\lambda +6
$$

令$det(B)=0, 计算 \lambda_1 = 3, \lambda_2 = 2$ 这是A所有的特征值

## finding all eigenvectors

分别将$\lambda_1 = 3, \lambda_2 = 2$ 分别带入到$(A-\lambda I)x = 0$中

$\lambda_1 = 3$

$$
\begin{bmatrix}
    1-3&-1\\2&4-3
\end{bmatrix}

\begin{bmatrix}
    x_1\\x_2
\end{bmatrix} = 

\begin{bmatrix}
    0\\0
\end{bmatrix}
$$

$$
\begin{bmatrix}
    -2&-1\\2&1
\end{bmatrix}

\begin{bmatrix}
    x_1\\x_2
\end{bmatrix} = 

\begin{bmatrix}
    0\\0
\end{bmatrix}
$$

所以只要满足$-2x_1 - x_2 =0$的x_1,x_2都是上述方程的解,　[1,-2]是$\lambda_1=3$对应的一组特征向量

$\lambda_1 = 2$带入

$$
\begin{bmatrix}
    1-2&-1\\2&4-2
\end{bmatrix}

\begin{bmatrix}
    x_1\\x_2
\end{bmatrix} = 

\begin{bmatrix}
    0\\0
\end{bmatrix}
$$

$$
\begin{bmatrix}
    -1&-1\\2&2
\end{bmatrix}

\begin{bmatrix}
    x_1\\x_2
\end{bmatrix} = 

\begin{bmatrix}
    0\\0
\end{bmatrix}
$$

所以只要满足$-x_1 - x_2 =0$的x_1,x_2都是上述方程的解,　[1,-１]是$\lambda_1=2$对应的一组特征向量

## 矩阵的迹和行列式

1. 矩阵的迹等于特征值之和$tr(A) = \lambda_1 + \lambda_2 + ... + \lambda_n$
2. 矩阵的行列式等于特征值的积$det(A) = \lambda_1  \lambda_2  ...  \lambda_n$

## OpenCV 计算特征值和特征矩阵

对于对称矩阵

```cpp
#include <opencv.hpp>
 
int main(void){
 
	double myArray[3][3] = {
		2, 1, 0,
		1, 3, 1,
		0, 1, 2
	};
 
	cv::Mat myMat = cv::Mat(3, 3, CV_64FC1, myArray);
	cv::Mat eValuesMat;
	cv::Mat eVectorsMat;
 
	std::cout << "My Mat: " << std::endl;
	for(auto i=0; i<myMat.rows; i++){
		for(auto j=0; j<myMat.cols; j++){
			std::cout << myMat.at<double>(i,j) << " ";
		}
		std::cout << std::endl;
	}
 
	cv::eigen(myMat, eValuesMat, eVectorsMat);
 
	std::cout << "Eigen Values : " << std::endl;
	for(auto i=0; i<eValuesMat.rows; i++){
		for(auto j=0; j<eValuesMat.cols; j++){
			std::cout << eValuesMat.at<double>(i,j) << " ";
		}
		std::cout << std::endl;
	}
 
	std::cout << "Eigen Vectors: " << std::endl;
	for(auto i=0; i<eVectorsMat.rows; i++){
		for(auto j=0; j<eVectorsMat.cols; j++){
			std::cout << eVectorsMat.at<double>(i,j) << " ";
		}
		std::cout << std::endl;
	}
 
	return 0;
 
}
```

对于非对称矩阵

Note: Use cv::eigenNonSymmetric for calculation of real eigenvalues and eigenvectors of non-symmetric matrix.

```cpp
void cv::eigenNonSymmetric	(	InputArray 	src,
OutputArray 	eigenvalues,
OutputArray 	eigenvectors 
)	
```

## Eigen 计算特征值和特征向量

```cpp
#include <iostream>
#include <Eigen/Dense>
#include <Eigen/Eigenvalues>
using namespace Eigen;
using namespace std;

void Eig()
{
    Matrix3d A;
    A << 1, 2, 3, 4, 5, 6, 7, 8, 9;
    cout << "Here is a 3x3 matrix, A:" << endl << A << endl << endl;
    EigenSolver<Matrix3d> es(A);

    Matrix3d D = es.pseudoEigenvalueMatrix();
    Matrix3d V = es.pseudoEigenvectors();
    cout << "The pseudo-eigenvalue matrix D is:" << endl << D << endl;
    cout << "The pseudo-eigenvector matrix V is:" << endl << V << endl;
    cout << "Finally, V * D * V^(-1) = " << endl << V * D * V.inverse() << endl;

    // 最大特征值和最小特征值与它的索引
    int col_index, row_index;
    cout << D.maxCoeff(&row_index, &col_index) << endl;
    cout << row_index << " " << col_index << endl;
}
int main()
{
	   Eig();
}

```
