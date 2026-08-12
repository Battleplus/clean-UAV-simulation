# 4 kg 机械臂力矩扰动观测器候选（默认关闭）

## 为什么需要新候选

已有两类预测补偿均未通过确定性配对 A/B：

- 机械臂模型反作用力矩前馈没有同时改善水平和高度窗口；
- 静态 COM gain `0.5` 在确定性复测中把水平漂移从 `0.072 m`
  恶化到 `0.151 m`，同时最大倾角从 `0.985 deg` 增加到
  `1.315 deg`。

因此不能继续提高预测前馈增益，也不能把 PX4 速度误差再积分一次冒充
扰动观测器。PX4 已有速度积分环，重复积分会增加低频振荡风险。

## 观测器结构

新增节点：

```text
drone_arm_sim/arm_disturbance_observer.py
```

它不读取 Gazebo truth，只使用可对应到实机的信号：

- PX4 `SensorCombined.gyro_rad`（FRD）；
- 实际发送到八个电机的归一化命令；
- CAD 电机位置、推力轴和当前调试推力映射；
- 机械臂在线计算的整机惯量和 COM 变化；
- 显式机械臂动作心跳与 PX4 解锁状态。

按刚体欧拉方程计算残余力矩：

```text
tau_dist = I * alpha + omega x (I * omega) - tau_motor
```

其中 `tau_motor` 使用动态 COM 修正后的 CAD 力臂计算。机械臂静止且飞机已
解锁时，观测器慢速学习电机模型、Q/T 临时值、IMU 偏置等静态残余；机械臂
开始动作后冻结基线，只发布相对于基线的增量残余。

安全门控：

- 未解锁、failsafe、数据陈旧、陀螺裁剪或异常采样周期：输出零；
- 没有机械臂动作心跳：输出零并只更新基线；
- 动作前至少积累 `0.2 s` 解锁静态基线；
- 角加速度先做一阶低通并限幅；
- 输出力矩范数默认限制为 `0.08 N m`；
- 电机补偿增量默认限制为每台 `1.0 N`；
- 观测器与旧预测前馈禁止在同一运行中同时启用；
- 所有开关默认关闭。

实验环境变量：

```text
ARM_TORQUE_FEEDFORWARD_ENABLED=false
ARM_DISTURBANCE_OBSERVER_ENABLED=true
ARM_DISTURBANCE_OBSERVER_GAIN=0.5
ARM_DISTURBANCE_OBSERVER_MAX_TORQUE_NM=0.08
ARM_DISTURBANCE_OBSERVER_MAX_DELTA_N=1.0
```

新的确定性执行器使用 `ARM_DOB_AB_GAIN` 记录候选增益，默认从 `0.25` 开始，
并拒绝非有限值、零/负值及大于 `0.5` 的直接尝试。旧无效运行中的 `0.5` 不再
被当作默认起点；只有较低增益通过相同配对门后才讨论下一步。

## 当前验证

- 观测器纯函数、动态 COM 力臂、欧拉方程、基线门控和限幅均有回归测试；
- 当前动力、4 kg、WASD 与 A/B 比较器联合回归：`81 passed`；
- Python 编译、shell 语法检查通过；
- `colcon build --symlink-install --packages-select drone_arm_sim` 通过；
- 当前 GUI 仿真上执行了 12 秒只观测冒烟测试；飞机未解锁时
  `active=false`、补偿输出始终为零，原始传感器残余约为毫牛米量级；
- 曾执行固定 seed、干净 PX4 工作目录的完整伸展/收回飞行对照，但复核发现
  ON 组观测器进程在启动时退出，因此该配对不构成有效算法 A/B；
- 当前手动 4 kg 联调继续运行默认关闭版本，没有注入该候选。

## 2026-08-11 配对运行复核：无效，必须重测

2026-08-11 使用固定 `GZ_RANDOM_SEED=4027`、全新 PX4 工作目录、起飞前
连续落稳门和同一条 `full_extend_slow_4kg` 执行了 OFF/ON 两次飞行。后续逐行
审计 ON 组 Gazebo 日志发现：

```text
arm_disturbance_observer: error: unrecognized arguments: --ros-args
process has died ... exit code 2
```

而且整份日志没有任何 `ARM_DOB_STATE` 发布标记。因此下面数值只描述两次飞行，
不能归因于观测器，更不能据此断言观测器改善或恶化了飞行：

| 指标 | observer OFF | observer ON，gain 0.5 | ON-OFF |
|---|---:|---:|---:|
| 动作窗口水平漂移 | `0.130 m` | `0.759 m` | `+0.629 m` |
| 动作窗口高度跨度 | `0.167 m` | `0.208 m` | `+0.041 m` |
| 最大真实倾角 | `1.703 deg` | `4.342 deg` | `+2.639 deg` |
| RMS 真实倾角 | `0.500 deg` | `1.030 deg` | `+0.530 deg` |
| 电机饱和 | `0/150` | `0/146` | 均为零 |
| PX4 failsafe | 无 | 无 | - |
| 完整伸展/收回到位 | 是 | 是 | - |
| 正常落地解除武装 | 是 | 是 | - |
| 严格飞行门 | 通过 | 失败 | - |

ON 组虽然完成动作并正常落地，但观测器节点根本没有运行；电机模型收到的候选
输入没有有效的实时观测器输出。严格比较器现将这组证据标记为
`candidate_runtime_valid=false`、`effect_evaluated=false` 和
`candidate_accepted=false`。这里的 `false` 表示证据无效，不代表算法已被科学否决。

原始证据：

```text
analysis/arm_dob_ab_20260811_v2_off_flight.log
analysis/arm_dob_ab_20260811_v2_on_g050_flight.log
analysis/arm_dob_ab_20260811_v2_comparison.json
```

该结论只适用于当前 4 kg 调试动力假设；不能外推到正式 `7.735 kg`，也不能
替代 4/5/7/8 号反向螺距桨的实物证据。

已修复观测器对 ROS 2 `--ros-args` 的解析，并增加运行时发布标记/进程存活硬门。
修复后在当前 4 kg 未解锁联合会话中以 ROS 2 节点重映射参数执行了限时冒烟测试；
节点正常启动并连续发布 `ARM_DOB_STATE`，`active=false`、输出为零，没有向默认关闭
的电机补偿分支注入控制量。
在新的有效配对 A/B 完成前仍默认关闭。当前状态：
`FLIGHT_AB_INVALID_RUNTIME_RETEST_REQUIRED_DEFAULT_OFF`。

2026-08-12 的后续模型审计又修正了两处观测器输入：耦合监视器现在发布当前
整机完整 `3×3` 质心惯量张量，观测器按 `I·α + ω×(I·ω)` 使用惯量积，不再
只保留对角项。张量消息显式标记为 `base_link_flu`，进入观测器前按
`I_FRD = R·I_FLU·Rᵀ` 转成 PX4 FRD，确保惯量积符号正确；同时观测器使用与
Gazebo 电机模型相同的上升/下降一阶响应后
再预测电机力矩，避免把执行器滞后误判成机械臂扰动。静态基线门也由
`样本数×最后一次 dt` 改成真实累计有效时长。另有确定性刚体闭环回归验证
观测正外扰后施加负补偿的符号能够降低姿态稳态/RMS 误差。相关聚焦回归为
`11 passed`。
这些修正仍需新的有效 OFF/ON 配对实飞验证，不能仅凭单元测试启用默认补偿。

完整张量离线复核使用显式 `--target-mass-kg 4.0` 重新生成，避免早期分析脚本
写死 7.735 kg 造成文件名与内部质量不一致。正确的 4 kg 结果显示：收回状态
惯量积范数约 `0.002805 kg·m²`，静态 `work_a` 增至 `0.006355 kg·m²`，加入
0.05 kg 刚性负载后增至 `0.007530 kg·m²`；对应 COM 偏移约
`0.000955 / 0.009959 / 0.010718 m`。运动 `work_a` 的模型反作用力矩约
`0.096346 N·m`。因此完整张量不是固定或冗余字段，机械臂姿态和负载确实改变
惯量积。机器数据见
`analysis/arm_coupling_full_tensor_4kg_modelcheck_20260812.json`。
独立校验器 `scripts/validate_arm_coupling_tensor_report.py` 会拒绝质量误标、非对称
或非正定张量、姿态不改变张量、负载不改变张量以及缺少有效运动反作用力矩的
报告。当前 4.00/4.05 kg 报告通过，结果保存在同名 `.validation.json`；错误
7.735 kg 冒充 4 kg 的合成回归会被拒绝，校验测试为 `2 passed`。

## 确定性 A/B 执行器

已增加：

```text
scripts/run_deterministic_arm_dob_ab.sh
```

执行器固定 `GZ_RANDOM_SEED=4027`、全新 PX4 工作目录、模型/估计器稳定门、
`full_extend_slow_4kg` 完整 90 秒伸展与 90 秒收回轨迹，并为 OFF/ON 两组
分别保存飞行、Gazebo、PX4、agent 和稳定门日志。它只有在显式设置
`RUN_DOB_AB_CONFIRM=1` 时才允许停止当前 GUI；无确认的干运行已验证返回
`64`，不会干扰手动联调。

比较器现在把以下条件同时纳入候选接受门：

- ON 组后端日志必须存在 `ARM_DOB_STATE`，且不得出现观测器进程退出或 CLI 参数错误；
- 机械臂动作中必须至少出现一次 `active=true` 和有限非零观测力矩；只有启动后
  全程 inactive/零输出的运行不能用于把飞行差异归因于观测器；

- 水平漂移 `<= 0.15 m`；
- 高度跨度 `<= 0.30 m`；
- 最大姿态倾角 `<= 3 deg`；
- 机械臂反作用力矩 `<= 0.50 N m`；
- 电机饱和样本为零；
- OFF/ON 都到达 `demo_extended` 和 `retracted`，正常 PASS；
- ON 同时不劣于 OFF 的水平漂移、高度跨度、最大倾角和 RMS 倾角。

比较器使用 `--require-improvement` 时，任何一项不满足都会返回非零退出码，
因此不能靠只改善单一指标或放宽门槛接受观测器。

## 2026-08-12 live data-path recheck

The current workspace-local 4 kg GUI/PX4 session was inspected without
enabling compensation or interrupting the user's controller.  PX4 had one
publisher on `/fmu/out/sensor_combined`; a live sample contained finite gyro
and accelerometer values, and `/my_drone/command/motor_speed` was measured at
approximately `250.073 Hz`.

The rebuilt observer was then run for eight seconds as a separate passive
node with ROS 2's appended `--ros-args`.  It started without a CLI error and
published repeated `ARM_DOB_STATE` records.  Because the aircraft was
disarmed and the arm was static, every record correctly reported
`active=false` and a zero estimated disturbance output, while the finite raw
residual changed with sensor input.  This proves the repaired parser and the
disarmed live input/output path; it is **not** an armed-motion effectiveness
test and does not replace the pending deterministic OFF/ON flight pair.

An allocation-level regression now starts from the analyzed 4 kg hover
commands, applies a finite three-axis observer torque, and checks the actual
eight-rotor wrench.  It requires the incremental force to remain zero, the
incremental torque to equal the negative disturbance in PX4 FRD, and every
motor to remain strictly inside its command limits.  This catches a reversed
compensation sign or an incorrect frame transform at the production allocator
boundary.

After this check, the complete package regression command
`python3 -m pytest -q -p no:cacheprovider src/drone_arm_sim/test
src/px4_ros2_control/test` reported `105 passed` with three upstream protobuf
deprecation warnings.  The observer remains disabled by default.

The next deterministic runner also writes one case manifest before each
backend starts.  It SHA-256 pins the 4 kg motor configuration, URDF, world,
PX4 airframe, backend launcher, arm-flight driver, observer source, direct
motor-model source, coupling-monitor source and the exact PX4 SITL executable,
together with the fixed seed, trajectory and all compensation limits.  The A/B comparator requires
the OFF and ON manifests to have identical `common` content and to differ only
in `observer_enabled=false/true`; it re-hashes every pinned source file at
comparison time.  A missing file, post-run source mutation or any hidden
gain/startup difference invalidates the pair before effect metrics are
evaluated.  The manifest writer and comparison gates report `9 passed`.
