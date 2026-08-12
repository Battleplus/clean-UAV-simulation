# Base 1 Phase D：分配前总 wrench 覆盖层审计

日期：2026-08-13

## 范围

- 唯一飞行基线：`base-1` / `b340ed6`；
- 唯一质量配置：4.000 kg；
- 7.0/7.735 kg 未读取、未启动、未测试；
- Base 1 的 13 个冻结飞行核心文件未修改，校验结果仍为 `BASE1_CORE_MATCH=true`；
- 本阶段不评价任何非零补偿增益，只建立安全的控制入口并验证零补偿非回归。

## 实现架构

```text
/my_drone/command/motor_speed（PX4 Base 1 原始 8 路命令）
                         ↓
          base1_wrench_reallocator
      原命令 → 推力 → Base 1 原 6D wrench
      Base wrench + 有界机械臂补偿 wrench
                         ↓
             一次受约束 6×8 分配
                         ↓
/my_drone/base1_compensated/command/motor_speed
                         ↓
       唯一 gazebo_direct_motor_model
```

新增文件：

- `drone_arm_sim/base1_wrench_reallocator.py`：数值内核和 ROS 2 覆盖节点；
- `scripts/activate_base1_wrench_reallocator_overlay.sh`：仅允许 4 kg Base 1、仅在地面/未解锁阶段接入覆盖层；
- `test/test_base1_wrench_reallocator.py`：编号、坐标、符号、边界、回零和分配测试；
- `scripts/test_base1_wrench_overlay_guard.py`：4 kg 专用和话题隔离静态保护。

Base 1 原 `gazebo_direct_motor_model.py` 和原 launch 均未修改。旧的分配后电机增量路径在该覆盖层中被明确关闭；覆盖层只使用一次总 wrench 分配。

## 安全行为

1. `BASE1_COMPENSATION_ENABLED=false` 时直接转发原 ROS 消息，不执行重分配；
2. 开关为真但三个增益均为 0 时，补偿 wrench 恒为 0，仍直接转发；
3. 估计状态无效、所需数据源超过 120 ms、PX4 状态超过 500 ms、飞机未解锁/不在 Offboard，或任一电机上下余量不足时，目标补偿变为 0；
4. 已施加补偿按力/力矩 slew rate 平滑回零，不保持最后一次补偿；
5. 求解失败、结果非有限或总 wrench 残差超过 0.02 时，立即清零内部补偿并转发 Base 1 原命令；
6. 单电机推力变化默认限制为 ±0.50 N，且始终受 `[0, maximum_thrust_n]` 物理边界约束；
7. 启动脚本要求原始电机话题、Gazebo odometry 和唯一原电机节点都存在；替代输出话题建立后才停止原节点，最终只允许一个 Gazebo 施力节点。

## 数值验证

9 项新增测试全部通过：

- PX4 1～8 号与配置列顺序双向映射；
- 命令/推力双向映射；
- 零补偿还原相同 Base wrench 和相同电机命令；
- 非零 6D 补偿通过一次受约束分配实现；
- FLU→FRD 和抵消符号；
- 过期目标逐步衰减至精确零；
- 未解锁、非 Offboard 或 PX4 状态过期时 fail-closed；
- 覆盖脚本只接受 4 kg 配置；
- 原始和补偿话题隔离，旧前馈路径关闭。

分配计算基准（2000 次）：

```text
mean = 0.374 ms
p50  = 0.333 ms
p99  = 1.020 ms
max  = 1.790 ms
p99 等效频率约 980 Hz
```

包构建成功；在补齐 PX4 消息环境后，包内回归 `80 passed`、脚本回归 `27 passed`，总计 `107 passed`；其中本阶段新增专项测试为 9 项。仓库原 `colcon test` 配置仍因未注册 pytest 测试而返回 `NO TESTS RAN`，不将其误记为代码失败或通过。

## 零补偿实飞验证

### A. 覆盖层关闭

日志：`base1_phase_d_relay_off_fresh_flight.log`

```text
DDS_DYNAMIC_HOVER_PASS
最大水平误差             0.0520 m
真实高度峰峰值           0.0280 m
PX4 垂向速度 P90         0.0040 m/s
真实垂向速度 P90         0.0096 m/s
电机饱和率               0
failsafe                  false
LAND / 解除武装           已确认
```

### B. 覆盖层开启、三个增益全为零

日志：`base1_phase_d_enabled_zero_flight.log`

```text
DDS_DYNAMIC_HOVER_PASS
最大水平误差             0.0758 m
真实高度峰峰值           0.0410 m
PX4 垂向速度 P90         0.0084 m/s
真实垂向速度 P90         0.0148 m/s
电机饱和率               0
failsafe                  false
LAND / 解除武装           已确认
```

两轮均使用原 Base 1 稳定门：水平速度不超过 0.10 m/s、垂向速度不超过 0.08 m/s，并要求连续稳定 10 秒；未放宽任何门限。

## 保留的无效尝试

首次覆盖层尝试因脚本在 ROS setup 期间启用 `set -u`，在接入前退出。后续驱动误启动的是未经过覆盖层的 Base 1，已主动中止，且不计为通过。修复只是在 source ROS 环境时临时关闭 nounset，没有修改任何飞行参数。

## 本阶段结论

Phase D 的入口和零补偿安全性已通过。当前默认仍为补偿关闭；这不等于机械臂补偿算法已经改善飞行。下一阶段必须按既定顺序分别测试平动力、静态重力矩和动态反作用力矩，并从 `0.05` 开始做配对 A/B。
