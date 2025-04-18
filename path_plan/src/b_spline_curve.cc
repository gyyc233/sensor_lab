#include "b_spline_curve.h"

double baseFunction(int i, int k, double u, vector<double> node_vector) {
  double Bik_u;
  // 0次B样条（1阶B样条）
  if (k == 1) {
    if (u >= node_vector[i] && u < node_vector[i + 1]) {
      Bik_u = 1;
    } else {
      Bik_u = 0;
    }
  } else {
    double denominator_1 = node_vector[i + k - 1] - node_vector[i];
    double denominator_2 = node_vector[i + k] - node_vector[i + 1];

    //# 如果遇到分母为 0的情况：
    //# 1. 如果此时分子也为0，约定这一项整体为0；
    //# 2. 如果此时分子不为0，则约定分母为1 。
    if (denominator_1 == 0)
      denominator_1 = 1;

    if (denominator_2 == 0)
      denominator_2 = 1;

    Bik_u = (u - node_vector[i]) / denominator_1 *
                baseFunction(i, k - 1, u, node_vector) +
            (node_vector[i + k] - u) / denominator_2 *
                baseFunction(i + 1, k - 1, u, node_vector);
  }

  return Bik_u;
}

vector<double> u_quasi_uniform(int n, int k) {
  vector<double> node_vector(
      n + k + 1); //准均匀B样条的节点向量计算，共n+1个控制顶点，k-1次B样条，k阶
  double piecewise = n - k + 2; // B样条曲线的段数:控制点个数-次数
  if (piecewise == 1) {         //只有一段曲线时，n = k-1
    for (int i = n + 1; i < n + k + 1; i++)
      node_vector[i] = 1;
  } else {
    //中间段内节点均匀分布：两端共2k个节点，中间还剩(n+k+1-2k=n-k+1）个节点
    for (int i = 0; i < n - k + 1; i++) {
      node_vector[k + i] = node_vector[k + i - 1] + 1 / piecewise;
    }
    for (int i = n + 1; i < n + k + 1; i++)
      node_vector[i] = 1; //末尾重复度k
  }
  return node_vector;
}

vector<double> u_piecewise_B_Spline(int n, int k) {
  vector<double> node_vector(n + k + 1);
  if (n % (k - 1) == 0 && (k - 1) > 0) { //满足n是k-1的整数倍且k-1为正整数
    for (int i = n + 1; i < n + k + 1; i++)
      node_vector[i] = 1;        //末尾n+1到n+k+1的数重复
    int piecewise = n / (k - 1); //设定内节点的值
    if (piecewise > 1) {
      //内节点重复k-1次
      for (int i = 1; i < piecewise; i++) {
        for (int j = 0; j < k - 1; j++)
          node_vector[(k - 1) * i + j + 1] = i / piecewise;
      }
    }
  } else {
    cout << "error!需要满足n是k-1的整数倍且k-1为正整数" << endl;
  }
  return node_vector;
}
