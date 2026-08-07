# 基于 Gazebo 与 ROS 2/PX4 SITL 的 8 旋翼异构全驱动空中操纵系统仿真控制方案

> 来源：飞书 wiki（用户 2026-07-19 粘贴整理）
> 说明：本文按原文结构整理，公式保留 LaTeX 写法，ASCII 架构图原样保留于代码块；文末列出整理时发现的复制错误与缺失项。

---

## 1. 8 旋翼异构全驱动 UAM 动力学建模与 PX4 控制分配（Control Allocation）

空中操纵系统（Unmanned Aerial Manipulators, UAM）是由多旋翼浮动基座与多自由度关节机械臂构成的强耦合、非线性浮动基座多体系统。本方案针对 **8 旋翼异构全驱动多轴飞行器** 与挂载于其底盘前下方的 **Hugging Face LeRobot SO-100 机械臂（5-DOF 关节 + 1-DOF 夹爪，共 6 轴）** 进行了整体动力学建模。

### 1.1 系统整体动力学方程

利用 Euler-Lagrange 形式，将 8 旋翼浮动基座与 6 轴 LeRobot 机械臂作为一个整体多体刚体树进行统一建模：

$$
M(q)\ddot{q} + C(q, \dot{q})\dot{q} + g(q) = \tau_{act} + \tau_{ext}
$$

其中：

- $q = \begin{bmatrix} p_W^\top & \eta^\top & \theta^\top \end{bmatrix}^\top \in \mathbb{R}^{12}$ 为系统的广义状态向量；
- $p_W = [x, y, z]^\top$ 为 8 旋翼基座在世界惯性系 $\mathcal{F}_W$ 中的质心位置；
- $\eta = [\phi, \theta, \psi]^\top$ 为机身姿态（欧拉角）；
- $\theta = [\theta_1, \theta_2, \theta_3, \theta_4, \theta_5, \theta_6]^\top \in \mathbb{R}^6$ 为 LeRobot SO-100 机械臂各物理关节角；
- $M(q) \in \mathbb{R}^{12 \times 12}$ 为系统质量与惯性矩阵；
- $C(q, \dot{q})$ 为科氏力及向心力矩阵；
- $g(q)$ 为重力向量。

控制输入 $\tau_{act}$ 包含 8 旋翼基座产生的合外力/力矩以及机械臂的关节轴向驱动扭矩：

$$
\tau_{act} = \begin{bmatrix} \mathbf{\Gamma}_B \mathbf{f} \\ \mathbf{\tau}_{arm} \end{bmatrix}
$$

其中：

- $\mathbf{f} = [f_1, f_2, \dots, f_8]^\top \in \mathbb{R}^8$ 为 8 个电机的电调输入推力矢量；
- $\mathbf{\tau}_{arm} \in \mathbb{R}^6$ 为机械臂关节驱动扭矩；
- $\mathbf{\Gamma}_B \in \mathbb{R}^{6 \times 8}$ 为 8 旋翼异构基座的控制分配矩阵。

### 1.2 PX4 v1.14+ 动态控制分配矩阵（Control Allocation Matrix）

在 PX4 v1.14 及其后续版本中，PX4 引入了现代 `control_allocator` 模块（完全替代了早期的静态混控器 Mixer）。该模块可对任意倾斜旋翼（Tilted Rotors）和过驱动/全驱动几何构型进行在线动态控制分配求解。

对于 8 旋翼异构全驱动无人机，每个旋翼 $i$ 的安装位置为 $\mathbf{r}_i = [x_i, y_i, z_i]^\top$，其推力方向单位矢量为 $\mathbf{n}_i = [u_{x,i}, u_{y,i}, u_{z,i}]^\top$。控制分配矩阵 $\mathbf{\Gamma}_B$ 映射关系定义为：

$$
\mathbf{\Gamma}_B = \begin{bmatrix}
k_f \mathbf{n}_1 & \dots & k_f \mathbf{n}_8 \\
k_f (\mathbf{r}_1 \times \mathbf{n}_1) + \sigma_1 k_\tau \mathbf{n}_1 & \dots & k_f (\mathbf{r}_8 \times \mathbf{n}_8) + \sigma_8 k_\tau \mathbf{n}_8
\end{bmatrix}
$$

其中：

- $k_f$ 为推力系数；
- $k_\tau$ 为阻力矩系数；
- $\sigma_i \in \{1, -1\}$ 表示第 $i$ 个电机的旋转方向（CW/CCW）。

由于 8 旋翼的控制分配矩阵 $\mathbf{\Gamma}_B$ 具有满秩特性（$\text{rank}(\mathbf{\Gamma}_B) = 6$），该平台具备在机身保持绝对水平的条件下，产生任意方向水平剪切推力抵消机械臂挥动和接触作业产生的反作用力矩（Reaction Wrench）的能力：

- **非全驱动基座（常规 4 轴）**：机械臂运动产生 $\tau_y$ 侧向力时，机身必须首先向侧向倾斜 $\phi$ 角，通过重力分量抵消该剪切力，控制带宽通常低于 5 Hz。
- **本方案的 8 旋翼全驱动飞控**：当 LeRobot SO-100（其 Feetech STS3215 舵机可输出高达 30 kg·cm 的高频动态力矩）向前伸展并对基座施加反作用力矩时，PX4 的 `control_allocator` 会在维持当前姿态环不变的前提下，通过零空间优化 $N(\mathbf{\Gamma}_B)$ 瞬时高频（控制带宽达 100 Hz）调配 8 个电机的差分转速，直接产生相反方向的剪切力，实现高精度原位悬停操作。

---

## 2. 基于 PX4 SITL 与 ros2_control 的联合仿真架构设计

本系统采用双控制总线管线并行的软件架构，利用 Gazebo Sim 的锁步机制（Lockstep Scheduler）进行高精度物理时钟同步，彻底解决飞控状态解算器与仿真环境时钟不一致造成的数值发散问题。

```
+-----------------------------------------------------------------------------------------+
|                                    Gazebo 仿真物理世界                                   |
|                                                                                         |
|       +------------------------------------+   +-----------------------------------+    |
|       |         8旋翼异构多旋翼刚体        |   |      LeRobot SO-100 关节刚体链     |    |
|       |                                    |   |                                   |    |
|       | - 8x 物理电机动力学插件            |   | - 6x 关节碰撞体/传动限位          |    |
|       | - 3D IMU & GPS 传感器物理仿真      |   | - 关节力矩阻尼解算                |    |
|       +-----------------|------------------+   +-----------------|-----------------+    |
+-------------------------|----------------------------------------|----------------------+
                          | (Actuator Speed / IMU Data)            | (Joint Trajectory / Force)
                          |                  | [gz_ros2_control]
                          v                                        v
+--------------------------------------------+   +----------------------------------------+
|              PX4 SITL 固件运行             |   |             ros2_control 框架          |
|  - 状态估计器 (EKF2)                       |   |  - JointTrajectoryController           |
|  - 姿态/位置环几何控制 (Offboard)          |   |  - GripperCommandController            |
|  - 动态 8 旋翼全驱动控制分配算法            |   |  - 手眼相机 & 全局 RGB-D 驱动          |
+---------------------|----------------------+   +--------------------|-------------------+
                      | (Micro XRCE-DDS)                              | (ROS2 Topic/Action)
                      v                                               v
+-----------------------------------------------------------------------------------------+
|                                  ROS 2 Humble / 具身智能栈                              |
|                                                                                         |
|      +----------------------------------------------------------------------------+     |
|      |                         Hugging Face LeRobot 决策端                        |     |
|      |  - ACT (Action Chunk with Transformer) / Diffusion Policy 离线训练与推理   |     |
|      |  - 实时手眼视觉闭环控制 & 运动路径生成 (MoveIt 2)                          |     |
|      +----------------------------------------------------------------------------+     |
+-----------------------------------------------------------------------------------------+
```

### 2.1 飞控控制闭环：PX4 SITL 与 Gazebo

1. **传感器数据回传（输入）**：Gazebo 仿真引擎中的物理惯导和定位插件（`gz-sim-imu-system`, `gz-sim-sensors-system`）高频产生惯导和 GPS 报文，通过 `px4_gz_bridge` 插件在本地 loopback 网络下以 UDP（端口 14560）将封包发送至本地运行的 PX4 SITL 实例中。
2. **控制量下发（输出）**：PX4 低层姿态控制器与控制分配器解算出 8 路电机的期望转速，封装为 `actuator_outputs`，高频（250 Hz – 400 Hz）发送至 Gazebo。Gazebo 的多旋翼电机模型插件（`MulticopterMotorModel`）订阅该期望转速并作用于物理旋翼关节上，产生真实的物理推力。

### 2.2 机械臂控制闭环：LeRobot 具身推理与 ros2_control

1. **ros2_control 驱动接口**：在仿真中，Gazebo 加载 `gz_ros2_control` 系统插件，并根据 SO-100 的 URDF 描述文件暴露出对应的关节力、速度和位置接口。
2. **MoveIt 2 与 LeRobot 的交互**：
   - Hugging Face 的 `lerobot` Python 策略推理器（如 ACT 策略）订阅 `/camera/hand_eye/image_raw` 视觉图像和 `/joint_states` 关节角度。
   - 推理器高频输出目标关节空间或端效应器空间轨迹，通过 `FollowJointTrajectory` 动作接口发送给 ros2_control 的 `joint_trajectory_controller`，高频驱动 Gazebo 内的机械臂运动。

### 2.3 飞控与机器人系统间通信：Micro XRCE-DDS

PX4 内部的 `uxrce_dds_client` 运行时与 ROS 2 Humble 环境中运行的 Micro-XRCE-DDS-Agent 建立高速 UDP 端口（默认为 8888）通信。这使得 PX4 的内部 uORB 消息可以直接和 ROS 2 Humble 节点的话题进行发布与订阅转换：

- **状态估计反馈**：ROS 2 的自主决策层、避障规划器 AM-Planner 订阅来自 PX4 的 `/fmu/out/vehicle_odometry` 获取当前的高频惯导/里程计状态。
- **位置/姿态指令注入**：高层节点（如自主协同控制层）通过发布 `/fmu/in/trajectory_setpoint` 向 PX4 飞控直接发送 3D 期望位置、速度，或发布 `/fmu/in/vehicle_rates_setpoint` 发送期望角速度，实现高机动的 Offboard 控制。

---

## 3. 基于 RoboStack 与 Conda 的全套软件环境配置 SOP

由于具身智能算法（PyTorch, CUDA 驱动）与低层机器人控制工具链（ROS 2, PX4）通常存在庞杂的 Python 和系统库版本依赖冲突，本 SOP 采用 Conda 作为唯一的虚拟隔离环境，在无需系统 root 权限的前提下快速完成高阶环境一键部署。

### 3.1 创建环境与软件源配置

打开终端，首先在你的 Linux 主机上下载并配置轻量化 Mamba（Miniforge）编译器：

```bash
curl -L -O "https://github.com/conda-forge/miniforge/releases/latest/download/Miniforge3-$(uname)-$(uname -m).sh"
bash Miniforge3-$(uname)-$(uname -m).sh -b -p $HOME/miniforge3
eval "$($HOME/miniforge3/bin/conda shell.bash hook)"
mamba init
```

重新打开终端，创建专用的 Conda 环境 `uam_px4_env`（强制指定 Python 3.10），并剔除 default 源，绑定 RoboStack 编译源：

```bash
mamba create -n uam_px4_env python=3.10 -y
mamba activate uam_px4_env

# 严格配置 Channel 顺序，防止 apt 依赖污染
conda config --env --add channels conda-forge
conda config --env --add channels robostack-humble
conda config --env --remove channels defaults
```

### 3.2 安装 ROS 2、MoveIt 2、Gazebo Sim 及其依赖

一键安装 ROS 2 Humble 核心、Gazebo 仿真连接包和 `gz_ros2_control` 接口：

```bash
mamba install ros-humble-desktop \
              ros-humble-ros-gz-sim \
              ros-humble-ros-gz-bridge \
              ros-humble-gz-ros2-control \
              ros-humble-moveit \
              ros-dev-tools \
              compilers \
              cmake \
              pkg-config \
              -y
```

安装 OpenGL 渲染驱动，避免仿真物理引擎加载 3D 模型时发生着色器崩溃：

```bash
mamba install -c conda-forge libgl-devel -y
```

### 3.3 在同一环境中安装 PyTorch (CUDA) 与 Hugging Face LeRobot

```bash
# 激活你的 CUDA 环境 (以显卡支持 CUDA 11.8 为例)
mamba install pytorch torchvision pytorch-cuda=11.8 -c pytorch -c nvidia -y

# 克隆并本地链接安装 LeRobot 生态库
cd ~
git clone https://github.com/huggingface/lerobot.git
cd lerobot
pip install -e "."
```

### 3.4 部署并编译 PX4 Autopilot 飞控 SITL 链

在 Conda 环境之外（或单独工作空间中）配置 PX4 飞控代码的编译工具链：

```bash
cd ~
# 克隆官方 PX4 固件
git clone https://github.com/PX4/PX4-Autopilot.git --recursive
cd PX4-Autopilot

# 运行 PX4 官方提供的依赖安装脚本
./Tools/setup/ubuntu.sh
# 重启计算机使系统依赖项完全生效
# 首次编译 PX4 固件以验证环境（这里以官方 x500 经典多轴模型为例）
make px4_sitl gz_x500
```

### 3.5 创建 ROS 2 空中操纵系统工作空间并编译

```bash
mkdir -p ~/uam_ws/src
cd ~/uam_ws/src

# 1. 克隆 LeRobot 官方 SO-100 机械臂 ROS 2 控制包
git clone https://github.com/brukg/SO-100-arm.git

# 2. 克隆 SO-100 总线舵机物理硬件接口 C++ 驱动插件 (用于 Sim-to-Real 实物部署)
git clone https://github.com/brukg/so_arm_100_hardware.git

cd ~/uam_ws
# 初始化并加载依赖项
rosdep init && rosdep update
rosdep install --from-paths src --ignore-src -r -y --rosdistro humble

# 采用符号链接开发模式编译
colcon build --symlink-install --cmake-args "-DCMAKE_BUILD_TYPE=Release"
```

---

## 4. PX4 8 旋翼异构多旋翼机型配置与模型装配（URDF/Xacro）

为了使你原生的 8 旋翼异构飞控能够准确识别并控制该构型，需要在 PX4 固件中增加自定义空气动力学机型（Airframe）参数，并在 Xacro 中将 8 轴推力矢量与 6 轴 LeRobot 机械臂刚体树完美拼接。

### 4.1 PX4 异构机型自定义空气动力学配置

在你的 PX4 代码库 `~/PX4-Autopilot` 下：

1. **创建 Airframe 配置文件**：在 `ROMFS/px4fmu_common/init.d-posix/airframes/` 下创建一个名为 `4015_gz_hetero_octorotor` 的机型参数定义文件（在 QGroundControl 的机型列表中注册为你自己的异构八轴）：

   ```bash
   touch ROMFS/px4fmu_common/init.d-posix/airframes/4015_gz_hetero_octorotor
   ```

2. **写入核心机型控制分配参数**：

   ```sh
   #!/bin/sh
   @name Heterogeneous 8-Rotor Omnidirectional UAM Base
   @type Octorotor x
   @class Octorotor
   @output MAIN1 motor 1
   @output MAIN2 motor 2
   @output MAIN3 motor 3
   @output MAIN4 motor 4
   @output MAIN5 motor 5
   @output MAIN6 motor 6
   @output MAIN7 motor 7
   @output MAIN8 motor 8
   . postconfig
   # 设置默认空气动力学控制参数
   param set-default CA_AIRFRAME 3
   param set-default CA_ROTOR_COUNT 8

   # 配置电机异构位置及非共面倾斜角 (示例：电机1前置、向左偏转15度、向上倾角)
   param set-default CA_ROTOR0_PX 0.25
   param set-default CA_ROTOR0_PY 0.25
   param set-default CA_ROTOR0_PZ 0.0
   param set-default CA_ROTOR0_AX 0.258
   param set-default CA_ROTOR0_AY 0.0
   param set-default CA_ROTOR0_AZ -0.965
   param set-default CA_ROTOR0_KM 0.016

   # 重复配置其余 7 个电机... (设置对应异构空间坐标轴 PX/PY/PZ 与姿态矢量 AX/AY/AZ)
   ```

3. **注册机型编译依赖**：编辑 `ROMFS/px4fmu_common/init.d-posix/airframes/CMakeLists.txt`，将 `4015_gz_hetero_octorotor` 添加到 `px4_add_romfs_files` 函数中。

### 4.2 8 旋翼异构 UAM 刚体树拼接（Xacro）

在 `~/uam_ws/src/SO-100-arm/so_arm_100_description/urdf/` 下创建整体机器人 Xacro 模型文件 `hetero_uam.urdf.xacro`。通过如下语法，实现 8 旋翼基座、LeRobot SO-100 刚体、ros2_control 控制插件和两路 RGB-D 深度相机的全局拓扑绑定：

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="hetero_uam">
  <xacro:include filename="$(find hetero_octorotor_description)/urdf/hetero_base_mesh.urdf.xacro" />
  <xacro:include filename="$(find so_arm_100_description)/urdf/so_100_arm.urdf.xacro" />

  <joint name="uav_to_arm_attachment_joint" type="fixed">
    <parent link="uav_base_link"/>
    <child link="so_100_base_link"/>
    <origin xyz="0.18 0.0 -0.12" rpy="0.0 0.0 0.0"/>
  </joint>

  <ros2_control name="GzSimUamSystem" type="system">
    <hardware>
      <plugin>gz_ros2_control/GazeboSimSystem</plugin>
    </hardware>
    <xacro:configure_so100_joints />
  </ros2_control>

  <link name="hand_eye_camera_link">
    <inertial>
      <mass value="0.05"/>
      <inertia ixx="1e-5" ixy="0" ixz="0" iyy="1e-5" iyz="0" izz="1e-5"/>
    </inertial>
  </link>

  <joint name="gripper_to_camera_joint" type="fixed">
    <parent link="wrist_roll_link"/>
    <child link="hand_eye_camera_link"/>
    <origin xyz="0.04 0.0 0.02" rpy="0.0 0.0 0.0"/>
  </joint>

  <gazebo reference="hand_eye_camera_link">
    <sensor type="depth" name="hand_eye_camera">
      <update_rate>30</update_rate>
      <visualize>true</visualize>
      <camera>
        <horizontal_fov>1.20</horizontal_fov>
        <image>
          <width>640</width>
          <height>480</height>
          <format>R8G8B8</format>
        </image>
        <clip>
          <near>0.05</near>
          <far>4.0</far>
        </clip>
      </camera>
    </sensor>
  </gazebo>
</robot>
```

---

## 5. 控制仿真系统一键启动与闭环测试运行

在物理仿真和具身智能交互中，整体链路可以通过以下指令序列和时钟同步桥接策略进行一键运行：

### 5.1 配置系统的 ROS-Gazebo 全局消息桥接文件

为了打通 PX4、Gazebo、ros2_control 以及双深度摄像头，在工作空间中建立桥接规则文件 `bridge_config.yaml`：

```yaml
# 桥接无人机的 IMU 与 EKF Odom 状态，回传给 Offboard 算法节点
- topic_name: "/model/hetero_uam/odometry"
  ros_type_name: "nav_msgs/msg/Odometry"
  gz_type_name: "gz.msgs.Odometry"
  direction: GZ_TO_ROS

# 桥接手眼相机捕获的 RGB 彩色图，回传给 LeRobot 深度学习推理端
- topic_name: "/camera/hand_eye/image"
  ros_type_name: "sensor_msgs/msg/Image"
  gz_type_name: "gz.msgs.Image"
  direction: GZ_TO_ROS

# 桥接机械臂关节状态，保证 MoveIt 2 和 LeRobot 能实时订阅当前连杆位姿
- topic_name: "/world/default/model/hetero_uam/joint_state"
  ros_type_name: "sensor_msgs/msg/JointState"
  gz_type_name: "gz.msgs.Model"
  direction: GZ_TO_ROS
```

### 5.2 仿真与控制闭环一键启动 SOP

请按照如下终端分发指令，严格执行系统的启动调试：

**第一步**：运行 Micro XRCE-DDS Agent，用于建立 PX4 与 ROS 2 的高带宽数据底座：

```bash
# 终端 1
MicroXRCEAgent udp4 -p 8888
```

**第二步**：启动你的 PX4 SITL 编译层，自动热加载你写好的异构 8 轴机型并唤醒 Gazebo Sim 物理环境：

```bash
# 终端 2
mamba activate uam_px4_env
cd ~/PX4-Autopilot
# 启动你在 ROMFS 注册的异构 8 旋翼 SITL
make px4_sitl gz_hetero_octorotor
```

**第三步**：激活你的 ROS 2 UAM 工作空间，加载 MoveIt 2 并唤醒 ros2_control 机械臂控制器：

```bash
# 终端 3
mamba activate uam_px4_env
source ~/uam_ws/install/setup.bash
# 唤醒针对 8 旋翼搭载的 SO-100 的轨迹规划引擎
ros2 launch so_arm_100 gz.launch.py dof:=5
```

**第四步**：启动 ROS-GZ Bridge，对齐无人机/关节/相机的双向消息通信流：

```bash
# 终端 4
mamba activate uam_px4_env
ros2 run ros_gz_bridge parameter_bridge --config-file ~/uam_ws/src/bridge_config.yaml
```

**第五步**：运行 LeRobot 收集数据集或部署具身模型进行闭环抓取测试：

```bash
# 终端 5
mamba activate uam_px4_env
# 在 Gazebo 物理中唤醒你的 LeRobot 推理客户端
lerobot-record --robot.type=so100_follower --dataset.num_episodes=50
```

---

## 6. 算力配置与硬件选型指南

本 UAM 系统具备过驱动 8 轴姿态高频物理循环、6 轴多体连杆 DART 精确碰撞力学解算、双深度 RGB-D 传感器实时光线追踪拟态、以及大模型 ACT (Action Chunk with Transformer) / SmolVLA 离线训练与低延迟推理等高负载特征。下面是根据任务计算负载定制的硬件配置选型方案：

> ⚠️ **原文缺失**：飞书文档中本节含一张「硬件选型配置表」，但粘贴内容仅显示「暂时无法在飞书文档外展示此内容」，表格未随文本导出。请补充表格（或截图）后我可补全本节。

针对本方案的技术约束深度分析：

1. **物理仿真锁步（Lockstep）时钟约束**：当在 Gazebo 中搭载了 8 旋翼电机高频 PID 驱动、飞控低层状态估计（EKF2 运行于 250Hz）以及 LeRobot 关节解算时，单核物理 CPU 主频是第一瓶颈。如果实时因子（Real-Time Factor, RTF）因单核瓶颈掉下 1.0，PX4 飞控会自动启动锁步同步暂停 Gazebo 的时钟向前滚动；但是如果此时 MoveIt 2 等第三方高层位置节点使用的是主机硬件「挂钟（Wall Clock）」而非仿真发布的 `/clock` 时钟，会导致计算出的位置控制量在时序上发生错位，导致多轴机架直接在空中因指令阶跃突变而「物理爆炸」坠毁。所以务必采用高性能高单核主频的 CPU。

2. **具身大模型闭环推理（Sim-to-Policy）延迟约束**：由于 ACT、SmolVLA 模型具有大量的自注意力机制与时序 Chunk 动作预测计算，推理前向传播时间（Inference Time）要求必须控制在 10 ms 以内。如果在没有 RTX Tensor 核心或者显存严重不足的机器上运行，GPU 会频繁进行显存与系统内存的 Swap 交换，导致推理时间暴涨至 150 ms 以上，全驱动 8 旋翼在物理上无法及时得到前馈反力扭矩控制信号，机械臂在空中执行推拉、阀门旋转等接触作业时，整个飞行平台会产生剧烈且无法收敛的极限环振荡。对于科学研究和高并发仿真，显存不低于 24 GB 的 RTX 4090 或 RTX 5080 是必须的。

---

## 7. 结论与未来空地协同系统展望

基于 PX4 Autopilot SITL 飞控固件、Micro XRCE-DDS 桥接和 Hugging Face LeRobot 具身智能框架，本技术方案为你的 8 旋翼异构全驱动多旋翼飞行器与 SO-100 开源轻量化机械臂建立了最严谨、最贴近物理实物的数字孪生仿真系统。

8 旋翼全驱动提供的 6 自由度独立解耦力和力矩输出（依靠 passively tilted 倾斜动力学），为克服空中机械臂重心漂移与操作力的干扰提供了完美抗扰机制；而 LeRobot 提供的 ACT 模仿学习大模型策略，则赋予了该操纵端无与伦比的高泛化、高语义理解作业能力。在 Conda+RoboStack 环境中，你可以高精度地在此仿真底座上离线生成数据集并完成策略评估，这将为你下一步物理实物的「零样本迁移（Zero-shot Sim-to-Real）」提供最为坚实的算法保证。

---

## 附：整理时发现的复制错误与缺失项

| 位置 | 原文问题 | 本次处理 |
|------|----------|----------|
| 1.1 广义状态向量 $q$ | 原文 `$q =^\top \in \mathbb{R}^{12}$` 缺失向量内容 | 重建为 $q = [p_W^\top, \eta^\top, \theta^\top]^\top \in \mathbb{R}^{12}$ |
| 3.1 Miniforge 安装 | 第二行缺 `bash` 前缀；`shell. hook` 缺 `bash` | 修正为 `bash Miniforge3-...sh -b -p ...` 与 `shell.bash hook` |
| 3.3 克隆 LeRobot | 「# 克隆…库cd ~」注释与命令粘连 | 拆分为注释行 + `cd ~` 命令 |
| 4.1 编号 | 步骤 1/2 之后夹了一个游离的 `touch ...`，又出现第二个「1. 写入…」 | 重新整理为 1/2/3 三步 |
| 5.2 第三步 | `source ~/uam_ws/install/setup.` 缺 `bash`；`dof:5` 应为 `dof:=5` | 修正为 `setup.bash` 与 `dof:=5` |
| 第 6 节 | 硬件选型表未随文本导出（显示「暂时无法在飞书文档外展示此内容」） | 标记为缺失，待补充 |
