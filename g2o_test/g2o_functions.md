- [顶点部分常见函数](#顶点部分常见函数)
- [边部分常见函数](#边部分常见函数)

## 顶点部分常见函数

1. oplusImpl(计算下一次的估计值，相当于一次setEstimate)：函数处理的是 xk+1 = xk + ∆x 的过程；
2. setToOriginImpl(清零：_estimate)：估计值清零；
3. setEstimate(设置：_estimate)：设置估计值，一般会有一个初始值；(在曲线拟合中有了估计值，也就可以算出y的估计值。)。
4. setId（设置顶点的id：_id）
5. estimate（ 返回： _estimate）：优化的结果。
6. addVertex（插入到_vertices，vertices()函数可以返回_vertices的值。）：添加顶点
7. setFixed：要优化的变量设置为false
8. setMarginalized ():设置该顶点是否被边缘化，_marginalized默认为false

## 边部分常见函数

1. computeError(返回：_error)：边的误差项，观测值与估计值的差距；(曲线拟合中y的测量值与估计值的差)。
2. linearizeOplus: 计算各个待优化变量的偏导数(jacobian矩阵)
3. setVertex（设置：_vertices）：设置连接的顶点
4. setMeasurement(设置：_measurement)：观测数据的输入，在曲线拟合中也就是y真实值(x值会在构建边的时候输入)；
5. setId：边的id
6. addEdge：v->edges().insert(e);添加边的数据，edges函数可以返回_edges的值
7. setInformation：设置信息矩阵(_information)
