# my_drone PX4 WASD 最终配置

## 一键启动

在 Windows PowerShell 中执行：

```powershell
powershell -ExecutionPolicy Bypass -File "E:\清洁无人机\drone_sim_ws\scripts\start_px4_wasd.ps1"
```

控制输入必须发送到启动脚本所在的 Windows Terminal，而不是 Gazebo
窗口。

## 按键

- `W/S`：前进 / 后退
- `A/D`：左移 / 右移
- `R/F`：上升 / 下降
- `Q/E`：左转 / 右转
- `Space`：锁定当前位置和高度
- `L` 或 `Esc`：降落并退出

## 实现说明

- 水平运动由 PX4 本地位置 Offboard 目标控制。
- 高度由 Gazebo `/model/my_drone/odometry` 世界 Z 闭环生成 PX4
  垂直速度目标，避免 PX4 相对高度的长期慢漂。
- 世界里程计超过 0.5 秒未更新时，垂直速度命令自动归零。
- 4015 airframe 使用 `SENS_IMU_MODE=0`、`EKF2_MULTI_IMU=1`，
  直接消费稳定的 `vehicle_imu`；磁罗盘仅用于初始化航向。
- PX4 飞行专用 `my_drone_octorotor_example.urdf` 将机械臂关节锁定在
  零位，避免 DART 关节约束造成的虚假 IMU 加速度尖峰。非 PX4
  机械臂运动学与动力学模型仍保持可动。

## 已完成的端到端验收

- W 前进：0.613 m
- D 右移：0.572 m
- S 后退：0.644 m
- A 左移：0.565 m
- 四段动作最大高度变化：0.002 m
- `L`：PX4 检测落地并正常退出

此外，可见 Windows Terminal 中的人工输入已经产生约 5 m 的真实世界
水平位移，同时高度保持在约 1.20 m。
