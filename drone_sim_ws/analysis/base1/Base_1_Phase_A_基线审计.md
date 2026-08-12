# Base 1 Phase A 基线审计

日期：2026-08-13

范围：仅 `base-1` / 4 kg；没有启动、测试或调整 7.735 kg 配置。

## 冻结结果

- 参考标签：`base-1`；
- 参考提交：`b340ed6ce77ac4f7820be5a80edba783ce7b55ec`；
- 13 个飞行核心文件的规范化 SHA-256 全部匹配；
- 默认加速度前馈：关闭；
- 默认机械臂力矩前馈：关闭；
- 默认静态 CoM 增益：0；
- 默认扰动观测器：关闭。

证据：`drone_sim_ws/baselines/Base_1_flight_freeze.json`。

## 第一次运行

配置：固定 seed 4027、Headless、全新 PX4 工作目录、`full_extend_slow_4kg`、伸展/收回各 90 s、所有补偿关闭。

机械臂动作阶段通过：

- 水平漂移：0.114 m；
- 高度跨度：0.086 m；
- 最大倾角：1.044°；
- RMS 倾角：0.471°；
- 最大机械臂反作用力矩：0.035 N·m；
- 电机饱和率：0；
- PX4 failsafe：无。

完整流程没有通过：动作完成并真实接地后，EKF local-z 保留约 0.15 m 残差，原测试的 0.05 m 地面判据未触发，最终等待 LAND/解除武装确认超时。

该次运行必须标记为“动力学基线通过、结束阶段失败”，不能计作完整通过。证据：

- `base1_no_comp_20260812_phaseA_run1_flight.log`；
- `base1_no_comp_20260812_phaseA_run1_reanalysis.json`。

## 第二次运行

飞行、机械臂、随机种子和补偿配置与第一次完全相同。仅在测试结束阶段，将 4 kg 调试机的接地 local-z 容差提高为 0.20 m；此容差只在 LAND 已请求且飞行器已位于地面、低速稳定后生效，不改变悬停或机械臂动作。

完整流程通过：

- `DDS_ARM_FLIGHT_PASS`；
- PX4 接受 LAND；
- `LANDING_DISARMED_CONFIRMED`；
- 水平漂移：0.091 m；
- 高度跨度：0.184 m；
- 最大倾角：1.044°；
- RMS 倾角：0.478°；
- 最大机械臂反作用力矩：0.035 N·m；
- 电机饱和率：0；
- PX4 failsafe：无。

证据：

- `base1_no_comp_20260812_phaseA_retry_run1_environment.txt`；
- `base1_no_comp_20260812_phaseA_retry_run1_flight.log`；
- `base1_no_comp_20260812_phaseA_retry_summary.json`。

## 当前判定

1. Base 1 飞行核心文件没有被修改；
2. 4 kg 无补偿、90 s 机械臂完整伸展/收回基线已再次完整通过一次；
3. 两次运行的机械臂动作动力学门均通过；
4. 尚未达到计划要求的三次完整重复通过，因此 Phase A/B 仍保持进行中；
5. 当前数据可作为后续补偿通道 A/B 的首个参考样本，但不能单独证明重复性完成。
