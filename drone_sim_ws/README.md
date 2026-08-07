# `my_drone` Gazebo / PX4 仿真工作区

本工作区沿用 WSL `/home/asus/drone_ws/drone_with_arm.urdf` 的机体和
SO-101 机械臂，Gazebo 实体名固定为 `my_drone`。

## 当前已完成

- URDF 正运动学、5 轴几何 Jacobian、阻尼最小二乘逆运动学；
- 随关节构型变化的整机质心和复合惯量；
- 无外力时机械臂运动引起的浮动基座瞬时反作用；
- NED/FRD 六自由度刚体仿真、理想 6D wrench 悬停控制；
- 满秩 6×8 推力分配、有界推力和 35 ms 一阶电机动力学；
- 保留原机体的 8 虚拟旋翼 URDF 和 Gazebo motor plugins；
- PX4 SITL 自定义 airframe 与 8 路执行器映射骨架。

当前旋翼几何和电机数据是验证用示例，不能当作实机参数。

## WSL 构建

```bash
cp -a '/mnt/e/清洁无人机/drone_sim_ws' ~/
cd ~/drone_sim_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

## 数值验证

```bash
ros2 run drone_arm_sim model_analysis
ros2 run drone_arm_sim inverse_kinematics \
  --from-joints shoulder_pan=0.4 \
  --from-joints shoulder_lift=-0.6 \
  --from-joints elbow_flex=0.8 \
  --from-joints wrist_flex=-0.5 \
  --from-joints wrist_roll=0.3
ros2 run drone_arm_sim floating_base_reaction
ros2 run drone_arm_sim allocation_analysis
ros2 run drone_arm_sim flight_control_demo --mode direct
ros2 run drone_arm_sim flight_control_demo --mode motors
```

## Gazebo

机械臂零重力轨迹测试：

```bash
ros2 launch drone_arm_sim arm_sim.launch.py
ros2 run drone_arm_sim trajectory_demo
```

直接 6D wrench 闭环悬停：

```bash
ros2 launch drone_arm_sim wrench_hover.launch.py
```

八旋翼电机链路测试：

```bash
ros2 launch drone_arm_sim octorotor_sim.launch.py
ros2 run drone_arm_sim motor_hover_demo
```

`motor_hover_demo` 只是开环电机/话题冒烟测试，不提供姿态稳定。PX4 接入步骤见
[`px4/README.md`](px4/README.md)，总体方案和卡点见
[`docs/IMPLEMENTATION_PLAN.md`](docs/IMPLEMENTATION_PLAN.md)。

完整的构建、自动测试和 Gazebo wrench 悬停验收可运行：

```bash
bash scripts/wsl_runtime_smoke_test.sh
```
