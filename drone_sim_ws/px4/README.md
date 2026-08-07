# PX4 SITL 接入

这里的 `4015_gz_my_drone_octorotor` 使用示例八旋翼参数，并与
`src/drone_arm_sim/config/octorotor_example.json` 保持同一套 PX4 FRD
坐标、电机编号和反扭矩符号。它已经在本机 PX4 SITL + Gazebo Harmonic
中完成 1 米相对高度 Offboard 悬停验收，但仍不是实机标定参数。

## 安装和编译

仅复制 airframe 文件还不够，必须同时把 `4015` 注册进 PX4 的
`airframes/CMakeLists.txt`。使用幂等安装脚本完成这两步并编译：

```bash
cd /home/asus/drone_sim_ws_codex
PX4_DIR=/home/asus/PX4-Autopilot \
  bash scripts/wsl_install_px4_airframe.sh
```

## 自动 PX4 悬停验收

测试会启动 `my_drone`，等待 PX4 EKF 进入 `Ready for takeoff`，解锁，
通过 MAVLink Offboard 持续发送本地 NED `z=-1 m` 位置目标，并验证世界
坐标约 `z=2 m`（初始生成高度为 `1 m`）：

```bash
cd /home/asus/drone_sim_ws_codex
bash scripts/wsl_px4_hover_test.sh
```

前、右、后、左连续水平航迹：

```bash
bash scripts/wsl_px4_direction_test.sh
```

该测试采用固定航向下的本地 NED 坐标：前=`north +1 m`、右=`east +1 m`、
后=`north -1 m`、左=`east -1 m`，并在每个航点检查位置误差和速度。

## WASD 手动控制

请在 Windows Terminal 中进入默认 WSL 后运行（需要真实交互终端）：

```bash
wsl -d Ubuntu-24.04
cd /home/asus/drone_sim_ws_codex
bash scripts/wsl_px4_wasd.sh
```

按键采用机头坐标：

- `W/S`：前进/后退
- `A/D`：左移/右移
- `R/F`：上升/下降
- `Q/E`：左转/右转
- `Space`：把当前位置设为新的悬停目标
- `L` 或 `Esc`：发送降落命令并退出

每次按键默认改变水平目标 `0.15 m`、高度目标 `0.10 m`、航向目标 `8°`；
长按依靠键盘自动重复形成连续移动。控制器始终以 20 Hz 发送 Offboard
位置目标，停止按键后会保持最后一个目标。使用时不要同时运行
`wsl_px4_hover_test.sh`、`wsl_px4_direction_test.sh` 或 QGroundControl，
避免争用 MAVLink `14550` 端口。

测试中的外力控制器只用于在 PX4 EKF 初始化阶段托住没有起落架的自由飞行
模型；进入 Offboard 前会被终止，此后唯一的飞行控制量来自 PX4 的 8 路
电机输出。

## 手动启动

终端 1：

```bash
cd /home/asus/drone_sim_ws_codex
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch drone_arm_sim octorotor_sim.launch.py
```

终端 2：

```bash
cd /home/asus/PX4-Autopilot
PX4_GZ_STANDALONE=1 \
PX4_GZ_WORLD=flight_world \
PX4_GZ_MODEL_NAME=my_drone \
PX4_SYS_AUTOSTART=4015 \
./build/px4_sitl_default/bin/px4
```

## 当前限制

- 旋翼位置、15° 倾角、推力系数、力矩系数和电机时间常数是仿真示例值。
- PX4 标准多旋翼位置控制主要使用垂直推力和姿态倾斜；还没有利用该构型
  的完整 6D 水平推力能力。
- Gazebo 气压计在 PX4 启动阶段仍偶发 `BARO STALE` 日志，但 GPS、IMU、
  磁罗盘、局部/全局位置最终有效，自动脚本会等待正式的
  `Ready for takeoff`，不会绕过健康检查。
- 机械臂静态质量、质心和关节动力学已参与飞行模型；机械臂运动中的 PX4
  抗扰动验收仍是下一阶段。
