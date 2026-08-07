# 7.735 kg 可飞基线冻结记录

本文件标记项目进入后续物理校准前的第一阶段：先保护已经验证过的可飞版本。后续实验（推力、反扭矩、电池、机械臂耦合或控制器参数）不得直接覆盖这组文件；需要新建配置或分支，并保留本基线可恢复。

## 基线内容

- 整机质量：`7.735 kg`
- 几何来源：`零件/` 中的 CAD 总装
- 正式 URDF：`drone_sim_ws/src/drone_arm_sim/urdf/my_drone_v3/my_drone_cad_formal_dynamic.urdf`
- 正式飞行配置：`drone_sim_ws/src/drone_arm_sim/config/my_drone_v3_cad_7p735_flight.json`
- PX4 airframe：`drone_sim_ws/px4/airframes/4026_gz_my_drone_octorotor_7p735`
- ROS 2/PX4 启动：`drone_sim_ws/scripts/wsl_start_ros2_dds.sh`
- WASD 控制：`drone_sim_ws/scripts/start_ros2_dds_wasd.ps1`
- SO101 控制：`drone_sim_ws/src/drone_arm_sim/drone_arm_sim/arm_preset_control.py`

## 已验收能力

- PX4 SITL + Gazebo + ROS 2 DDS 链路启动
- `T` 解锁起飞，`W/S/A/D` 前后左右移动，`Q/E` 偏航，`L` 降落
- SO101 收回姿态和安全飞行动作
- 静态回归测试 `20 passed`
- WASD 联动标志：`DDS_WASD_PTY_PASS`
- 机械臂安全飞行联动标志：`DDS_ARM_FLIGHT_PASS`

完整 `work_a/work_b` 机械臂动作仍可能耗尽当前推力余量并触发 failsafe，因此不属于本基线的飞行动作；它们只作为地面预设动作保留。

## 回退方式

本记录对应 Git 标签 `baseline-7p735-flyable`。开始任何后续实验前，先创建实验分支：

```bash
git switch -c experiment/<name> baseline-7p735-flyable
```

需要恢复时：

```bash
git switch --detach baseline-7p735-flyable
# 或从标签创建新的恢复分支
git switch -c recovery/7p735 baseline-7p735-flyable
```

标签对应的正式 URDF、PX4 配置和控制脚本必须保持完整，不得删除或原地改写。
