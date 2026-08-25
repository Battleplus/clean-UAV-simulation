# 1.3 kg / 0.6 kg 机械臂候选审计

日期：2026-08-20

## 质量口径

- 整机约 1.3 kg，包含完整机械臂。
- 机械臂约 0.6 kg，其余机体约 0.7 kg。
- 当前按原 CAD 体积分布分别缩放两组质量与惯量；不是逐件称重结果。
- Base 1（4 kg）与历史 7.735 kg 配置均未覆盖。

折叠姿态候选惯量对 Base 1 的比例为：

```text
Ixx: 0.2572
Iyy: 0.2659
Izz: 0.2650
```

因此 4028 候选采用独立的初始角速度环增益。该缩放只有惯量依据，尚未完成动态辨识。

## 已有动态证据

无机械臂动作的 PX4/WASD 测试通过：

- 最大稳定速度跟踪误差约 0.0347 m/s；
- 最大巡航高度误差约 0.069 m；
- 无电机饱和、PX4 failsafe；
- 正常 LAND 与解除武装。

原始日志：`wasd_flight_acceptance_1p3kg.log`。

继承 Base 1 角速度增益的机械臂方向测试未通过。向前伸出阶段稳定，但收回末段产生持续滚转振荡，最终触发安全降落。异常时机械臂已接近收回位置、估算 COM shift 接近零，因此不能用静态重心偏置单独解释。

保留日志：`directional_workspace_flight_acceptance_1p3kg_inherited_rate_fail.log`。

## 当前离线安全证据

- 标准静态扫描：25,725 个姿态。
- 允许飞行姿态：9,127 个。
- 最大允许 COM shift：0.05891 m。
- 最大允许静态重力矩：0.72858 N·m。
- 最小单电机补偿保留量：0.30011 N。
- 十方向端点覆盖检查全部通过。
- 十方向伸出、保持、完整回收均通过 81 点连续轨迹预演。
- 原最高“向上”端点路径存在 `upper_arm_link–wrist_link` 碰撞代理；规划器跳过 43 个碰撞候选后选择另一安全逆解分支。

机器审计状态：

```text
CANDIDATE_1P3KG_OFFLINE_READY_DYNAMIC_NOT_ACCEPTED
```

该状态明确不等于 Gazebo/PX4 动态通过。

## 自适应补偿边界

慢速自适应使用 PX4 原始电机命令重构的六维 effort residual，不使用位置、速度、姿态或角速度 P/D 生成额外 wrench，避免与 PX4 闭环重复。

保留的学习状态只有在以下条件全部成立时才可施加：

- 明确启用自适应；
- 动力学模型源数据新鲜；
- Gazebo truth 新鲜；
- PX4 处于 Armed/Offboard；
- 电机仍有规定余量。

任一条件失效时，学习状态保留但不再作为目标输入；实际补偿由统一 slew 连续退出。自适应默认关闭，必须完成严格 OFF/ON A/B 后才允许启用。

## 下一次动态执行顺序

1. 使用 4028 新角速度增益重新跑无臂起飞、悬停、WASD、H 和降落。
2. 运行 `bash scripts/run_front_retract_acceptance_1p3kg.sh`，只执行向前伸出、保持、收回，验证滚转振荡是否消失；日志独立写入 `front_retract_flight_acceptance_1p3kg.log`。
3. 依次运行左右、前后、上下和四对角方向。
4. 最后运行连续组合动作与慢速自适应 OFF/ON A/B。

所有动态阶段继续执行统一门限：XY/Z 峰峰值不超过 0.05 m、最大倾角不超过 1°、零饱和、零 failsafe、补偿无突归零且能够正常回收降落。

1.3 kg 候选入口固定使用 81 点连续轨迹预演。不得退回曾漏检中间碰撞的 41 点设置；`ARM_DIRECTIONAL_ONLY` 只裁剪本次实际执行腿，不改变完整十方向包线或计划证据。

## 2026-08-24 统一十方向回归

实验性组合为：重力矩补偿开启、测量差分得到的反作用力/力矩关闭、直接世界 XY 位置反馈开启，并让 PX4 在机械臂动作窗口接收零水平速度目标。原始证据：

- `directional_all10_unified_default_1p3kg.log`
- `directional_all10_unified_default_1p3kg.telemetry.jsonl`
- `directional_all10_unified_default_1p3kg.telemetry_report.json`

结果为 30 个阶段中的 29 个通过；十个方向中的九个完整通过。唯一失败是 `down/retract`：

```text
XY peak-to-peak = 0.720873 m
Z peak-to-peak  = 0.011232 m
max tilt        = 10.727937 deg
motor saturation = 0
PX4 failsafe      = false
```

这不是推力饱和，而是两个水平闭环争用：直接世界 XY 力反馈经倾斜旋翼产生平移力，同时 PX4 水平速度 PID 又通过倾斜机身反向纠偏。旧的 `PX4_TRUTH_HOLD_ARM_POSITION_OVERLAY` 只把 PX4 水平速度目标设为零，并没有旁路水平速度 PID，因此不能作为正式统一方案。

实验性直接位置反馈和旧零速度 overlay 已恢复为候选默认关闭。下一版必须使用明确的 mixed-axis 所有权：机械臂动作时 PX4 的 XY 位置/速度设点为 NaN、XY 加速度设点为零，Z 仍保留有效速度/高度控制；直接 XY 外环才成为唯一水平平移环。该模式必须具有稳定进入门、reallocator 健康门和小于 100 ms 的自动回退，完成针对 `down/retract` 复测后才能重跑全十方向。
