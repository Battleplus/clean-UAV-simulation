# `my_drone` 仿真运行状态（2026-07-30）

## 最终结论

默认 WSL `Ubuntu-24.04` 下已经打通：

1. SO-101 机械臂正/逆运动学、Jacobian、质量/质心/复合惯量；
2. 关节动力学与浮动基座动量反作用；
3. Gazebo DART 直接 6D wrench 悬停；
4. 8 个带时间常数的 Gazebo `MulticopterMotorModel` 闭环悬停；
5. PX4 SITL 自定义 `4015` 控制分配、解锁、起飞和 MAVLink Offboard
   相对 1 米悬停；
6. 固定航向下前、右、后、左 1 米水平航迹。

PX4 最终自动验收数据：

- 世界 ENU 位置：`[-0.0104, 0.0450, 2.1465] m`
- 目标：`[0, 0, 2] m`（初始生成高度 `1 m`，相对上升 `1 m`）
- 位置误差：`0.1536 m`
- 姿态误差：`3.166 deg`
- 线速度：`0.0111 m/s`
- 角速度：`0.00973 rad/s`
- PX4 状态：`Ready for takeoff`、`Armed`、`Takeoff detected`、
  `navigation mode: Offboard`

水平航迹各航点误差：

- 中心爬升：`0.0866 m`
- 前进（North +1 m）：`0.0651 m`
- 右移（East +1 m）：`0.0538 m`
- 后退（South 1 m）：`0.0262 m`
- 左移回中心（West 1 m）：`0.0155 m`

## 关键修正

- WSL 网络由 mirrored 改为 NAT，并通过 Windows 代理完成 PX4 递归浅克隆。
- 修复无效 maintainer 邮箱，使 `colcon` 正确识别 `ament_python`。
- Gazebo 物理引擎改为 DART；Bullet Featherstone 会使多关节模型发散。
- 瞬时 wrench 改为每个 1 ms 物理步发布，避免 persistent wrench 累积。
- 修复八旋翼生成器遗漏的 FRD→FLU 旋翼位置转换。
- 为 PX4 硬编码传感器接口保留根链接名 `base_link`。
- 将八旋翼 ROS/Gazebo `/clock` 桥改为只从 Gazebo 到 ROS，避免重复发布者。
- 增加 wrench bridge，使 PX4 EKF 初始化前的临时自由飞行支撑真正生效。
- 修复临时 wrench 控制器把机体系 odometry 线速度误当世界系速度的问题。
- 降低该倾斜八旋翼的 PX4 yaw 权重和 yaw rate 总增益，消除接管后的角振荡。
- 用持续 MAVLink Offboard 本地位置目标替代一次性 `commander takeoff`，
  避免 EKF 高度重置后提前进入 Hold。
- 将 `4015` 同时注册到 PX4 airframe CMake 清单；仅复制脚本不会进入 ROMFS。

## 验证命令

```bash
cd /home/asus/drone_sim_ws_codex

# 数值/静态回归
python3 -m pytest -q

# 直接 wrench、八电机和长时间 wrench 回归
bash scripts/wsl_runtime_smoke_test.sh
bash scripts/wsl_octorotor_smoke_test.sh
bash scripts/wsl_octorotor_wrench_smoke_test.sh

# PX4 端到端
bash scripts/wsl_px4_hover_test.sh
bash scripts/wsl_px4_direction_test.sh

# 交互式机头坐标 WASD 控制
bash scripts/wsl_px4_wasd.sh
```

## 尚存卡点

- `BARO STALE` 在 PX4 启动收敛期偶发，虽然最终 EKF 和悬停验收通过，仍应
  在追求确定性 CI 前继续检查 Gazebo air-pressure 回调与 PX4 lockstep 时序。
- 示例旋翼参数没有实机台架标定，不可直接用于真机。
- 当前 PX4 标准多旋翼控制没有利用倾斜八旋翼的完整 6D wrench 能力。
- 尚未在 PX4 悬停过程中执行机械臂轨迹；下一步应把关节动作作为质心和惯量
  时变扰动，验证 PX4 抗扰动或加入在线质心前馈。
