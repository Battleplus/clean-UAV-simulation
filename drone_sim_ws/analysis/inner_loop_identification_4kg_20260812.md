# 4 kg PX4 内环自动辨识记录（2026-08-12）

## 适用范围

本记录只适用于 `4027` / 4 kg 调试机型，不覆盖 7.735 kg 正式基线，
也不用于证明 4、5、7、8 号桨的真实正反桨配置。

## V2 原始飞行结果

- 原始协议报告：`inner_loop_identification_4kg_20260812_v2_raw.json`
- 原始 ULog：`/tmp/my_drone_px4_work.mi9NXN/log/2026-08-11/16_11_24.ulg`
- 重新生成的环路报告：`inner_loop_identification_4kg_20260812_v2_loops.json`
- 协议流程完成并正常落地、解除武装；无 failsafe，电机饱和比例为 `0`。
- 最大直接控制倾角为 `3.511 deg`，没有越过 `8 deg` 安全门。

流程通过不等于调参通过。V2 的可用测量为：

| 环路 | 轴 | 目标 | 峰值 | 超调 | 上升时间 | 稳定时间 | 判定 |
|---|---|---:|---:|---:|---:|---:|---|
| 姿态 | roll | 3.000° | 3.512° | 13.68% | 0.700 s | 未测出 | 不通过 |
| 姿态 | pitch | 3.000° | 3.510° | 12.71% | 0.752 s | 未测出 | 不通过 |
| 角速度 | pitch | 6.000°/s | 5.281°/s | 0% | 未达到 90% | 未测出 | 数据不完整 |
| 角速度 | yaw | 10.000°/s | 6.188°/s | 0% | 未达到 90% | 未测出 | 数据不完整 |

roll 角速度报告无效：PX4 切换到 body-rate 控制的日志窗口晚于零速基线，
分析器只看到非零平台和短暂回零，因而拒绝将其当作完整阶跃。这个缺口不能
用假设补齐，也不能据此修改 rate PID。

## V3 协议修正

V3 保持原来的位置恢复和 `8 deg` 安全门，但修改为：

- 每个直接控制阶段先连续发送 `0.80 s` 基线；
- 姿态阶跃平台延长到 `1.40 s`，回零 `0.60 s`；
- roll/pitch 角速度改为 `4 deg/s × 1.20 s`，累计命令角约 `4.8 deg`；
- yaw 角速度改为 `8 deg/s × 1.50 s`；
- rate 回零平台为 `0.20 s`；
- 协议参数写入每个 raw JSON 的 `protocol` 字段。

这样既给 PX4 模式切换留出明确零基线，又给 `0.30 s` 稳定带判定留下足够
观测时间，同时不靠提高激励幅度获得数据。

当前 V3 代码和分析器共 `10 passed`，`px4_ros2_control` 已重新构建。V3 实飞
必须在独占的干净 headless 后端运行，不能与用户当前打开的 GUI/WASD 联调
实例并行。V3 实飞完成前不修改 `MC_ROLL_P`、`MC_PITCH_P` 或 rate PID。
启动脚本还会主动检测 `gz sim gui` 和 `dds_wasd_control`；即使已设置第一层
确认变量，只要发现交互会话仍会以退出码 `65` 拒绝运行。只有明确设置
`RUN_INNER_LOOP_ID_TAKEOVER=1` 才允许清理交互后端，避免误杀用户正在测试的
联合仿真。

## 自动调参门

`scripts/derive_px4_tuning_candidate.py` 强制按角速度、姿态、速度的内到外顺序
检查。每个轴必须同时具有目标、峰值、上升时间、超调、稳定时间和稳态误差，
且全局电机饱和率不超过 `0.001`，否则不允许产生参数改动。当前 V2 报告得到：

```text
decision = HOLD_INCOMPLETE_IDENTIFICATION
active_layer = body_rate
parameter_changes = []
```

机器报告为 `analysis/inner_loop_tuning_gate_4kg_20260812_v2.json`。完整数据若超门，
脚本一次最多生成 `5%` 的单层实验候选；候选不会自动写入 airframe，必须在
一次性 4 kg 测试机型上复跑相同辨识协议后才能接受。聚焦回归为 `3 passed`。

内环 V3 与速度环使用两份独立证据：V3 ULog 提供角速度和姿态阶跃；速度层
必须通过 `--velocity-evidence` 传入由原始 10 Hz WASD 日志生成且带 SHA-256
的报告。缺少这份报告时，即使内环全部通过，调参门也会停在 `velocity`，
不会把 V3 中没有独立速度阶跃的数据误当成速度环辨识。

`scripts/build_px4_tuning_candidate_airframe.py` 负责把被授权的单次候选复制到
新的 4 kg airframe。它拒绝覆盖源文件、拒绝 4026/7.735 kg 名称、逐项核对
旧参数值、限制每次变化不超过 `5%`，并把源 airframe、调参决定和输出文件的
SHA-256 写入 manifest。`HOLD_*` 报告不能生成候选，正式基线不会被修改。

`scripts/run_inner_loop_identification_4kg.sh` 现已把上述阶段串成一次执行：实飞
结束后自动分析 ULog、生成分层调参决定；只有决定明确为
`BOUNDED_CANDIDATE_REQUIRES_RETEST` 时才生成独立候选 airframe 和 manifest。
候选不会自动应用，执行器会打印下一次复测所需的 `PROJECT_AIRFRAME_FILE`。
若提供 `VELOCITY_IDENTIFICATION_EVIDENCE`，文件必须存在才继续；缺少原始
10 Hz 速度证据仍会保持 HOLD。脚本语法、交互运行保护及候选/门控回归均已
通过；使用 V2 不完整报告复核仍正确停在 `body_rate`，没有生成参数候选。

新增 `scripts/run_layered_identification_4kg.sh` 作为内到外总执行器。它先运行
V3 角速度/姿态辨识并读取机器判定；只允许 `active_layer=velocity` 的结果继续
10 Hz 速度辨识。若 body-rate 或 attitude 数据不完整、超门或产生候选，流程
立即停在内层并要求按同一协议复测，不会提前调整速度参数。内外层都形成有效
证据后才生成合并决定，必要时建立新的独立 4 kg airframe。总执行器同样具有
GUI/WASD 防抢占门，当前会话上的保护测试返回 `65` 并通过。

## 机械臂收回静态门

基础飞行辨识不能只依赖 `arm_preset_control --wait` 的位置到达结果；关节仍在
收敛时产生的反作用力矩会污染角速度和姿态阶跃。启动链现新增
`scripts/wait_arm_static.py`，在机械臂到达 `retracted` 后继续要求六个 SO101
关节同时满足：

- 最大位置误差不超过 `0.08 rad`；
- 最大关节速度不超过 `0.03 rad/s`；
- 上述条件连续保持 `2 s`；
- `/joint_states` 数据保持新鲜且字段完整、有限。

缺失关节、缺失速度、非有限数据或任一条件超限都会拒绝启动 PX4 自动飞行。
三项纯函数回归通过；当前 4 kg GUI 实例上的只读实测为：

```text
ARM_STATIC_GATE_PASS preset=retracted samples=501
max_error_rad=0.000508 max_speed_rad_s=0.000000
```

因此后续 V3 的解锁前门顺序为：Gazebo 整机落稳 → 机械臂收回并静止 → PX4
局部速度估计连续稳定。该门不会把可动关节改成 `fixed`，但能证明辨识窗口开始
前机械臂处于受控静止状态；机械臂动态耦合仍留在后续阶梯试验中单独评估。

## 有限自动复测 campaign

`scripts/run_layered_tuning_campaign_4kg.sh` 将“生成候选后人工再次启动”扩展成
有限的自动复测闭环。一次用户确认后，它仍严格按 body-rate → attitude →
velocity 的顺序运行；每轮只消费机器判定明确生成的一次性 4 kg 候选，再用
完全相同的协议从头复测。默认最多 `4` 轮，硬上限 `6` 轮。

campaign 不会修改源 airframe，也拒绝文件名不含 `debug_4kg` 的输入和候选。
内环候选只接受总执行器预期的退出码 `2`，内外层合并候选只接受完整运行的
退出码 `0`；这样即使失败运行残留了 JSON 或候选文件，也不会被下一轮误用。
证据不完整、全局安全门失败、无候选 HOLD、候选缺失或达到迭代上限都会立即
停止。只有 `ALL_LAYERS_PASS_NO_PARAMETER_CHANGE` 才报告 campaign 通过，且
得到的仍是分析目录中的 4 kg 候选，不会提升或覆盖正式 7.735 kg 配置。

干运行、迭代范围和 GUI/WASD 防抢占门已通过 shell 回归；当前交互会话存在时
返回 `65`，因此 campaign 尚未实飞。

campaign 的控制流另用隔离临时目录和模拟分层执行器覆盖了三条关键路径：
内环候选（预期退出码 `2`）进入下一轮并最终 PASS；候选文件存在但执行器异常
退出时以 `candidate_from_failed_runtime` 拒绝；`HOLD_INCOMPLETE_IDENTIFICATION`
不会进入下一轮。结果为 `LAYERED_TUNING_CAMPAIGN_BRANCH_PASS`。模拟执行器只能
通过显式测试环境变量注入，生产默认仍固定调用真实分层辨识脚本。

速度证据提取器还增加了分阶段与整段聚合超调的一致性门。整段水平、垂直和
偏航超调可以因为跨轴耦合而高于对应按键阶段，但绝不能低于任一命令阶段的
超调；否则说明原始指标聚合存在矛盾，报告会标记
`overshoot_consistency_pass=false`，禁止进入自动调参。新增的故意低报总超调
回归被正确拒绝，提取器当前为 `4 passed`。
