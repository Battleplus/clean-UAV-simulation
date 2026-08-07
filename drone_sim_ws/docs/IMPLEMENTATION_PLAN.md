# 运动学、动力学与飞行仿真实施方案

## 1. 已确认的现有进度

- 系统：WSL2 Ubuntu 24.04。
- ROS：ROS 2 Jazzy。
- 仿真：Gazebo Harmonic；交互式 ROS 环境使用 vendor 版 `gz-sim 8.11`。
- 已安装：`ros_gz`、`ros_gz_bridge`、`gz_ros2_control`。
- 已完成过 ROS talker/listener、Gazebo `/clock`、ROS-Gazebo bridge、
  `diff_drive` 与简单盒子模型测试。
- 原始整机模型 `/home/asus/drone_ws/drone_with_arm.urdf`：
  9 个 link、8 个 joint，其中 5 个机械臂转动关节、1 个夹爪关节；
  总质量约 2.632006 kg。
- 该模型能被 URDF 和 SDFormat 解析，也曾被 Gazebo 成功创建为
  `my_drone`。

后续所有 Gazebo 实体、轨迹控制话题和状态话题均统一使用
`my_drone`；不替换现有机体与机械臂模型。

## 2. 当前模型的边界

现有模型只是“几何与惯量模型”，不是可控机器人：

1. 没有 `ros2_control`、transmission 或 Gazebo 控制插件；
2. 没有 launch、world、ROS package 和测试节点；
3. DART 日志显示 STL mesh collision 全部创建失败；
4. 没有关节 damping/friction；
5. 无旋翼 link、旋翼推力插件、电机参数、IMU 或飞控；
6. 所以“能显示/能生成实体”不等于“机械臂可控”，更不等于“能飞”。

## 3. 运动学建模

将无人机视为浮动基座，将机械臂视为串联链：

```text
T_WE(q) = T_WB · T_BA · Π_i [T_joint,i · R(axis_i, q_i)]
```

- `T_WB`：无人机基座在世界坐标系的位姿；
- `T_BA`：机械臂安装变换；
- `q_i`：SO-101 关节角；
- `T_WE`：末端执行器位姿。

机械臂几何 Jacobian：

```text
Jv_i = z_i × (p_E - p_i)
Jw_i = z_i
J = [Jv; Jw]
```

本工程的 `model_analysis` 已实现：

- URDF 树解析；
- 正运动学；
- 几何 Jacobian；
- 各 link 惯性坐标变换后的整机质心与复合惯量；
- 带关节限位的阻尼最小二乘逆运动学。

逆运动学采用：

```text
q_dot = Jᵀ (J Jᵀ + λ²I)⁻¹ v_E
```

避免在奇异位形附近直接求逆。

## 4. 动力学建模

完整浮动基座模型：

```text
[M_bb  M_ba] [v_dot_b] + C(q,v)v + g(q) = [W_rotor + W_ext]
[M_ab  M_aa] [q_ddot ]                    [tau_arm       ]
```

关键不是手写一条总方程，而是把以下数据填准确：

- 每个 link 的质量、质心、惯性张量；
- 关节轴、限位、阻尼、摩擦和最大扭矩；
- 机械臂安装位姿；
- 接触工具的碰撞体和接触参数。

实施分两层：

1. Gazebo/Bullet Featherstone 负责真实多体积分和接触；
2. 控制器侧后续使用 Pinocchio 计算 `M(q)`、`C(q,v)`、`g(q)`，
   与 Gazebo 输出对照验证。

目前先使用 Gazebo 原生 `JointTrajectoryController` 验证动力学链。
安装 `ros-jazzy-ros2-controllers` 后，再切换到
`gz_ros2_control` 的 effort/trajectory controller。

`floating_base_reaction` 已用有限差分得到各 link 相对速度，并根据总线动量和
角动量为零求解基座瞬时 twist；当前示例的动量残差达到浮点数数值零。Gazebo
动态对照仍未完成。

## 5. 飞行动力学与控制分配

单个旋翼：

```text
f_i = k_f · omega_i²
tau_drag,i = sigma_i · k_m · f_i
```

八旋翼合力/力矩：

```text
W_B = Gamma · f
Gamma_i = [n_i;
           r_i × n_i - KM_i · n_i]
```

`allocation_analysis` 已提供一个满秩 6×8 示例，并检查：

- `rank(Gamma) == 6`；
- 奇异值和条件数；
- 悬停推力是否全部非负；
- 伪逆分配残差。

`flight_control_demo` 已实现：

- PX4 风格世界 NED / 机体 FRD 坐标约定；
- 6DoF 刚体平动和转动积分；
- 理想 6D wrench 控制；
- 非负有界最小二乘控制分配；
- 八个旋翼 35 ms 一阶推力响应。

两种模式都已从位置和姿态扰动收敛到目标。八旋翼 URDF 已加入 8 个虚拟
rotor link、Gazebo `MulticopterMotorModel`、IMU、磁力计、气压计和 GPS。

示例参数不能直接用于实机。必须用真实的：

- 8 个旋翼中心坐标 `r_i`；
- 每个旋翼推力轴 `n_i`；
- CW/CCW 方向；
- `k_f`、`k_m`、最大/最小转速和电机时间常数。

## 6. 推荐迭代顺序

### M1：固定/零重力机械臂

- 模型生成成功；
- 六个关节能按轨迹运动；
- joint state 能返回 ROS 2；
- 不出现 NaN、爆炸或关节超限。

状态：静态模型、launch 和控制话题已完成；等待 WSL 恢复后动态复验。

### M2：正常重力、固定基座

- 用 world fixed joint 固定无人机基座；
- effort controller 抗重力；
- 验证静态重力矩和能量。

### M3：自由浮动基座

- 取消固定；
- 观察机械臂运动导致的基座反作用；
- 对比质心预测和 Gazebo 位姿。

状态：数值零动量模型已完成，Gazebo 对照待验证。

### M4：直接 wrench 悬停

- 暂不模拟单个旋翼；
- 给基座施加 6D wrench；
- 完成姿态、位置控制器和机械臂前馈补偿。

状态：独立 6DoF 数值仿真已完成；Gazebo `ApplyLinkWrench`、3D odometry、
ROS bridge、质心力矩补偿控制节点和自动悬停验收脚本也已生成，等待 WSL
动态运行。

### M5：八旋翼动力学

- 增加 8 个 rotor link 与 motor model；
- 使用 `Gamma` 将期望 wrench 分配为 8 路推力；
- 检查悬停、平移、姿态保持和饱和。

状态：数值闭环与 Gazebo 模型文件已完成；Gazebo 插件动态加载待验证。

### M6：接入 PX4

- 先运行官方 PX4 Gazebo 基线；
- 再注册自定义八旋翼模型和 airframe；
- 通过 uXRCE-DDS 接入 ROS 2；
- 最后增加机械臂质心/反作用 wrench 前馈。

状态：`4015_gz_my_drone_octorotor` airframe、FRD 几何和 8 路
`SIM_GZ_EC_FUNC` 已生成；PX4 仓库与 SITL 尚未在本机运行。

## 7. 坐标系边界

- Gazebo/ROS 模型使用 ENU 世界坐标与 FLU 机体坐标；
- PX4 使用 NED 世界坐标与 FRD 机体坐标；
- 旋翼 JSON 和 PX4 airframe 使用 FRD；
- URDF 生成脚本按 `[x, y, z]FLU = [x, -y, -z]FRD` 转换推力轴。

必须通过单桨方向测试确认推力方向、CW/CCW 反扭矩和 actuator 编号，不能只凭
矩阵静态一致性认定端到端方向正确。

## 8. 当前卡点

1. `sudo` 需要用户密码，暂时无法安装 `ros-jazzy-ros2-controllers`。
2. 原模型的碰撞体全部是高面数 STL；DART 明确报告无法创建。
   当前试验改用 Bullet Featherstone，正式接触仿真仍应制作简化碰撞体。
3. WSL 同时安装系统 Gazebo 8.14 和 ROS vendor Gazebo 8.11。
   运行 ROS 仿真时必须先 source Jazzy，保持整套库都使用 8.11，
   避免插件 ABI 混用。
4. 默认 Ubuntu-24.04 的 WSL 后端启动持续超时，本轮无法执行 colcon、Gazebo
   插件加载和 PX4 SITL。切换 `.wslconfig` 的 mirrored networking 会影响所有
   WSL 发行版，尚未擅自修改。
5. 无真实八旋翼几何与电机参数。按示例 `k_f` 和 PX4 omnicopter 同样的
   `1100 rad/s` 上限，单桨最大推力约 10.344 N；数值初始机动峰值约 9.98 N，
   裕度仍然很小，真实设计需要重新核算推重比和控制裕度。
6. WSL 中尚无 PX4-Autopilot 仓库，因此 airframe 只完成静态一致性检查。
   PX4 标准多旋翼控制链是否能充分利用独立水平推力也需要 SITL 试验；真正
   独立 6D wrench 可能需要自定义 thrust-setpoint 生成器。
7. 清洁末端工具、目标表面和接触力指标尚未定义。
