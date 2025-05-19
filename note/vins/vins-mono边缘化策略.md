
# VINS-MONO边缘化策略

在 VINS-Mono 中，边缘化（Marginalization）策略由变量 marginalization_flag 控制。它决定了当前帧是否是关键帧，并进一步影响滑动窗口中哪一帧被边缘化

## 边缘化最老帧

条件

- 当前帧是一个关键帧（Keyframe）(视差大)
- 滑动窗口已满（即 frame_count == WINDOW_SIZE）
- 系统决定保留最新帧，将最老帧（索引为 0 的帧）从优化问题中移除

实现方式

- 在 optimization() 函数中，会构造一个 drop_set，把 para_Pose[0] 和 para_SpeedBias[0] 加入其中
- 这些参数块对应的是 最老帧的 IMU 状态（位姿、速度偏置）
- 使用 Schur 补方法进行边缘化操作，只保留其对系统的影响作为先验项
- 同时，也会将与最老帧有关的视觉残差和 IMU 残差加入到 MarginalizationInfo 中

## 边缘化次新帧

条件

- 当前帧不是关键帧（Non-keyframe）
- 次新帧（倒数第二帧，索引为 WINDOW_SIZE - 1）将被边缘化
- 最新帧（WINDOW_SIZE）保留在窗口中

实现方式

- 构造 drop_set，仅包含 para_Pose[WINDOW_SIZE - 1]
- 不处理速度偏置（para_SpeedBias[WINDOW_SIZE - 1]），因为非关键帧不需要完整状态估计
- 执行预边缘化和正式边缘化
- 更新地址映射（addr_shift），让后续帧继承次新帧的状态
- 清理掉旧的边缘化信息并更新 last_marginalization_info
