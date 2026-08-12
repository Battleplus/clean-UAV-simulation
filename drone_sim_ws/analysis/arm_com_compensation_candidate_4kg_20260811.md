# 4 kg 动态重心补偿候选方案（尚未通过 A/B）

## 本轮修正

原实现把机械臂动态反作用力矩和静态重心偏置共用
`/my_drone/arm_motion_active` 门控。动作停止约 1 秒后，即使机械臂仍保持在
伸展姿态，静态重心补偿也会消失。这与真实重力矩持续存在的物理关系不符。

当前实现已改为两个独立门控：

- 动态反作用力矩：仅在机械臂运动心跳和反作用力矩数据都新鲜时使用；
- 静态重心力矩：只要求重心力矩数据新鲜，动作结束后仍持续使用；
- 两者分别具有增益，便于做可归因的 A/B；
- 最终仍通过同一套 6×8 控制分配矩阵，并受单电机补偿推力上限约束；
- 默认配置保持关闭，不能在 A/B 前提升为正式功能。

新增实验参数：

```text
ARM_TORQUE_FEEDFORWARD_ENABLED=true
ARM_REACTION_TORQUE_FEEDFORWARD_GAIN=0.0
ARM_STATIC_COM_FEEDFORWARD_GAIN=0.5
ARM_STATIC_COM_FEEDFORWARD_TIME_CONSTANT_S=5.0
ARM_TORQUE_FEEDFORWARD_MAX_DELTA_N=2.0
```

其中 `REACTION...=0.0` 用于第一轮只隔离静态重心补偿，避免把此前没有通过
A/B 的动态反作用力矩前馈混入结果。

## 离线安全性检查

4 kg、`demo_extended` 姿态的动力学计算结果：

```text
总质量                         4.000 kg
相对收回姿态的 COM 变化       [0.001259, 0.001374, -0.022927] m
水平 COM 偏移                  0.001863 m
静态重力矩 FLU                [-0.053882, 0.049379, 0.0] N m
gain=0.5 最大归一化电机改变量 0.004780
gain=1.0 最大归一化电机改变量 0.009560
```

这说明候选补偿很小，不会以大幅改变油门来掩盖控制问题；但离线有界不代表
飞行性能改善。

## 必须执行的验收

从干净的 4 kg headless 后端分别执行同一条
`full_extend_slow_4kg` 伸展/收回轨迹：

1. FF 全部关闭的同批基线；
2. 仅静态 COM、gain `0.5`；
3. 若第 2 组同时降低动作窗口姿态和高度波动，才测试 gain `1.0`；
4. 对比水平漂移、高度跨度、姿态峰值/均方根、电机饱和、COM 变化和最终
   收回后的恢复窗口；
5. 任一实验出现 failsafe、饱和、内部安全 LAND，或只改善一个指标而恶化
   另一个指标，均不得接受。

## 第一组飞行 A/B 结果

同一代码、同一 `full_extend_slow_4kg` 轨迹、每组干净启动：

| 项目 | FF 关闭 | 仅静态 COM gain=0.5 | 变化 |
|---|---:|---:|---:|
| 全窗口水平漂移 | 0.296 m | 0.141 m | -0.155 m |
| 高度跨度 | 0.127 m | 0.132 m | +0.005 m |
| 最大 truth 倾角 | 2.720 deg | 1.400 deg | -1.320 deg |
| 电机饱和 | 0/152 | 0/154 | 均为 0 |
| failsafe | 无 | 无 | — |
| 严格运行结果 | FAIL | PASS | — |

原始日志：

- `analysis/arm_flight_full_extend_com_ab_off_20260811.log`
- `analysis/arm_flight_full_extend_com_ab_on_g050_20260811.log`
- `analysis/arm_com_ab_comparison_g050_20260811.json`

本组数据表明水平漂移和倾角明显改善，但高度跨度增加 `5 mm`，而且关闭组
本次没有通过严格水平门槛。比较器因此正确给出
`paired_runs_accepted=false` 和
`feed-forward did not reduce both measured windows`。不能挑选更早的好基线，也
不能只根据对照组单次 PASS 把补偿设为默认；至少需要新的成对重复运行，且两组
都通过相同门槛，才能判断 gain `0.5` 的改进是否可重复。

## 确定性启动复测与最终判定

为消除上一组配对运行中 PX4 地面局部坐标原点相差 `0.433 m` 的混杂因素，
后端启动链路增加了三项可重复性约束：

- Gazebo 固定 `--seed 4027`；
- 自动测试使用全新的 PX4 工作目录，不复用旧 `parameters.bson`；
- 所有机械臂飞行 profile 在起飞前必须连续落稳 `5 s`。

两次独立的 4 kg 夹爪飞行均通过，PX4 地面 NED `z` 原点分别为
`0.023 m` 和 `0.014 m`，差值降为 `0.009 m`。对应的漂移差为
`0.013 m`、高度跨度差为 `0.016 m`，且两次均无饱和、无 failsafe、正常
落地解除武装。机器可读证据为
`analysis/flight_start_reproducibility_4kg_20260811.json`。

随后在上述相同启动约束下重新执行 `full_extend_slow_4kg` 配对 A/B：

| 项目 | FF 关闭 | 静态 COM gain=0.5 | 变化（开－关） |
|---|---:|---:|---:|
| 全窗口水平漂移 | 0.072 m | 0.151 m | +0.079 m |
| 高度跨度 | 0.174 m | 0.150 m | -0.024 m |
| 最大 truth 倾角 | 0.985 deg | 1.315 deg | +0.330 deg |
| 电机饱和 | 0/151 | 0/151 | 均为 0 |
| failsafe / 安全中止 | 无 | 无 | — |
| 相同门槛运行结果 | PASS | FAIL | — |

原始日志和比较结果：

- `analysis/arm_flight_full_extend_com_ab_deterministic_off_20260811.log`
- `analysis/arm_flight_full_extend_com_ab_deterministic_on_g050_20260811.log`
- `analysis/arm_com_ab_deterministic_g050_20260811.json`

静态 COM gain `0.5` 虽降低高度跨度，却同时恶化水平漂移和最大倾角，并越过
相同的 `0.15 m` 水平门槛。因此候选被正式否决，不继续测试 gain `1.0`，所有
补偿默认保持关闭。该结果只否决当前静态预测前馈实现，不否定后续受限扰动
观测器方案。

当前状态：`DETERMINISTIC_PAIRED_AB_REJECTED_DEFAULT_DISABLED`。
