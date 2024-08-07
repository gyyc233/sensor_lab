- [reference](#reference)
- [位姿优化误差函数介绍](#位姿优化误差函数介绍)


## reference

- ![视觉SLAM 十四讲——3D-2D:PnP求解——BA](https://www.cnblogs.com/bokeyuan-dlam/articles/15078888.html)
- ![视觉SLAM位姿优化时误差函数雅克比矩阵的计算](https://blog.csdn.net/u011178262/article/details/85016981)
- ![SLAM优化位姿时，误差函数的雅可比矩阵的推导。](https://blog.csdn.net/zhubaohua_bupt/article/details/74011005)

## 位姿优化误差函数介绍

在SLAM优化问题中，我们一般使用 李代数 ξ 来表示 旋转和平移。

![](./img/SLAM位姿优化时误差函数雅可比矩阵的计算/image1.png)

![](./img/SLAM位姿优化时误差函数雅可比矩阵的计算/image2.png)

那么误差函数可以表示为：

- 直接法，这里的2误差函数是关于像素值的函数
灰度不变假设：同一个空间点的像素灰度，在各个图像中是固定不变的。

![](./img/SLAM位姿优化时误差函数雅可比矩阵的计算/image3.png)

- 特征点法
最小化 重投影误差，即地图点到当前图像投影点与匹配点的坐标误差

![](./img/SLAM位姿优化时误差函数雅可比矩阵的计算/image4.png)

