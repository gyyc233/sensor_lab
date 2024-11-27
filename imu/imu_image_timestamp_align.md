
# imu & camera 时间戳同步

由于硬件系统的触发延时、传输延时和没有准确同步时钟等问题，IMU和相机之间通常存在时间偏差，估计并纠正这个偏差将有效提升VIO系统的性能

相机与IMU数据流时间戳上的偏差将影响VIO系统的工作，比如在VINS-Mono系统中，两个KeyFrame图像之间的IMU预积分项将因为时间不对齐而不准确，从而给状态估计带来偏差

IMU和相机时间偏差标定的几种方法包括:

1. [基于MSCKF的在线时间标定方法](https://intra.ece.ucr.edu/~mourikis/papers/Li2014IJRR_timing.pdf)
2. [Online Temporal Calibration for Monocular Visual-Inertial Systems](https://arxiv.org/abs/1808.00692)
3. [Kalibr 工具箱中离线的时间标定方法](https://github.com/ethz-asl/kalibr/wiki/camera-imu-calibration)

以下为绕沈劭劼团队方法进行学习

## Online Temporal Calibration for Monocular Visual-Inertial Systems

### 假设传感器之间的时间偏移是一个常数

以imu时间戳为参考时间，应根据 td 向前或向后移动相机图像序列，没有移动整个摄像机或 IMU 序列，而是移动特征点的观测值

- 下图描绘了一幅说明时间偏移的图片。在图中，上半部分表示采样瞬间，下半部分显示了时间戳瞬间。由于触发延迟、传输延迟和时钟不同步，生成的时间戳不等于实际采样时间，从而导致摄像机和IMU之间的时间错位
- 定义td: $t_{imu}=t_{cam}+td$, ，其中时间偏移 td 是我们应该移动摄像机时间戳的时间量，以便摄像机和 IMU 数据流在时间上保持一致。td 可以是正值或负值。如果相机延迟比 IMU 长，则 td 为负值。相反，td 为正值。

![](./img/imu_image_timestamp_align/img1.png)

### 引入用于建模和补偿时间错位的特征速度

假设: 在很短的时间内（几毫秒），摄像机的运动可以被视为匀速运动，故图像特征在短时间内在图像平面上也近似匀速移动.基于这个假设，我们计算特征在图像平面上的速度

![](./img/imu_image_timestamp_align/img2.png)

## 参考文章

- [传感器时间戳对齐](https://haolin11.github.io/2022/12/03/Online-Temporal-Calibration-for-Monocular-Visual-Inertial-Systems/)
- [VIO系统中相机和IMU时间戳对齐](https://blog.csdn.net/weixin_50508111/article/details/122466521)
