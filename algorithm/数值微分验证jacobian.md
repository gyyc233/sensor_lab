
## 比较理论雅可比与数值微分jacobian

```python
import numpy as np

def f(x):
    # 定义你的函数 f(x)
    pass

def J_theory(x):
    # 定义你理论推导的雅可比矩阵
    pass

def numerical_jacobian(f, x, epsilon=1e-6):
    n = len(x)
    m = len(f(x))
    J_num = np.zeros((m, n))
    for j in range(n):
        x1 = x.copy()
        x1[j] += epsilon
        diff = (f(x1) - f(x)) / epsilon
        J_num[:, j] = diff
    return J_num

# 选择一个验证点 x
x = np.array([1.0, 2.0, 3.0])

# 计算理论雅可比和数值雅可比
J_theo = J_theory(x)
J_num = numerical_jacobian(f, x)

# 打印和比较两者的差异
print("Theoretical Jacobian:\n", J_theo)
print("Numerical Jacobian:\n", J_num)
print("Difference:\n", J_theo - J_num)
```

关注误差矩阵 `J_theo - J_num` 是否足够小。如果误差在数值精度范围内 (例如 1e-5 到 1e-6 之间)，则理论雅可比矩阵是正确的

[具体实现](../task/test/gauss_newton_curve_fitting_test.cc)
