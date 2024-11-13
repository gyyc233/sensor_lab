- [common trajectory tracking algorithms](#common-trajectory-tracking-algorithms)
  - [based on vehicle kinematics](#based-on-vehicle-kinematics)
  - [based on vector](#based-on-vector)


自主移动机器人，或者说无人驾驶车辆，一般有三大子系统

- 环境感知子系统 perception
  - 通过传感器获取环境信息，主要包括环境感知、建图、定位
- 行为决策子系统 planning
  - 结合导航目标、环境信息、交通规则等信息规划出一条精密的行驶轨迹，无人车沿着这条轨迹移动就能到达终点
- 运动控制子系统
  - 结合行为决策子系统规划的轨迹和车辆当前状态，位置、姿态和速度等，得出无人车的控制量，油门、档位、转向等

## common trajectory tracking algorithms

### based on vehicle kinematics

- PID controller：采用预瞄控制思想，即以车辆前方预瞄点处的运动状态作为反馈设计PID
- LQR: 线性二次型调节器
- MPC

### based on vector

- pure pursuit 纯追踪
- Stanley 斯坦利轨迹跟踪：基于横向跟踪误差的非线性反馈函数进行跟踪
