# Gazebo 物理步施力修复与速度控制复测（2026-08-11）

## 根因

原 `gazebo_direct_motor_model.py` 在每次 ROS 里程计回调时向 Gazebo
`/world/flight_world/wrench` 发布一次 `EntityWrench`。Gazebo 8 的
`ApplyLinkWrench` 对该普通主题只施力一个物理步；ROS/Gazebo 桥接没有保证
每个 4 ms 物理步都到达一条消息，因此旧链路会出现短暂零推力。

不能直接改用 `/wrench/persistent`：Gazebo 8 源码会把每条 persistent 消息
追加到向量中，并在以后每个物理步把所有历史 wrench 相加，而不是覆盖同一
实体的旧值。

## 修复

- 新增 `drone_motor_system::LatestWrenchSystem` Gazebo C++ System 插件。
- ROS 侧发布 `/world/flight_world/wrench/latest`，每条消息覆盖缓存的最新值。
- 插件在 Gazebo `PreUpdate` 的每个物理步调用 `AddWorldWrench`。
- 1 s 墙钟超时作为控制进程失联保护；没有新消息时不会永久保留非零推力。
- 原普通和 persistent 主题继续保留给独立诊断工具，但飞行电机模型不再使用。

## ULog 对比证据

| 指标 | 旧瞬时 wrench | 新 physics-step latest wrench |
|---|---:|---:|
| 真值速度单样本最大跳变 | `0.2298 m/s` | `0.00363 m/s` |
| 真值最大垂直加速度 | `11.49 m/s²` | `0.181 m/s²` |
| `>5 m/s²` 垂直脉冲次数 | `8` | `0` |
| 估计/真值速度相关系数 | `0.9816` | `0.9994` |

证据文件：

- `analysis/vertical_velocity_ulog_with_accel_ff.json`
- `analysis/vertical_velocity_ulog_latest_wrench.json`

## 控制复测

### 垂直 R/F/R/H

- PX4 估计垂直峰值：`0.164 m/s`（相对 `0.15 m/s` 约 `9.3%`）
- Gazebo 真值峰值：`0.165 m/s`（`10%`）
- H 阶段 Gazebo 真值最大速度：`0.019 m/s`
- 最大横滚/俯仰：`0.4°`
- 电机饱和：`0`
- 无 failsafe，正常 LAND/解除武装

原始日志：`analysis/velocity_vertical_latest_wrench_scurve2.log`。

### 完整 W/S/A/D/R/F/Q/E/W/H

- 水平峰值：`0.422 m/s`，超调约 `5.4%`
- 垂直峰值：`0.147 m/s`，无超调
- 偏航峰值：`15.4°/s`，超调约 `2.7%`
- 最大 Gazebo 横滚/俯仰：`2.5°`（包含水平加速阶段）
- 电机饱和：`0`
- 无 failsafe，正常 LAND/解除武装
- 标志：`DDS_VELOCITY_WASD_PASS`

原始日志：`analysis/velocity_full_latest_wrench_scurve.log`。

### 独立 Q/E/H

- 偏航峰值：`15.4°/s`
- 最大非指令垂直速度：`0.027 m/s`
- 最大 Gazebo 横滚/俯仰：`0.4°`
- 电机饱和：`0`
- 标志：`DDS_VELOCITY_WASD_PASS`

原始日志：`analysis/velocity_yaw_latest_wrench.log`。

## 结论边界

本次结果证明 4 kg 调试机型的动力执行链断帧已消除，并通过无机械臂速度和
偏航联合控制。它不证明 7.735 kg 正式机已经具备足够推力余量；4、5、7、8
是否为反向螺距桨以及真实反扭矩仍需厂家/实物/推力台证据冻结。在这些物理
事实完成前，`0.994` 与 `2.08` 仍是两种假设。
