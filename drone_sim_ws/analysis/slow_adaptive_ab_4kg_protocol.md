# Base 1 慢速自适应严格 A/B 协议

当前状态：**NOT_RUN**。本协议和自动判定器已经建立，但编写时没有启动或停止
Gazebo/PX4，也没有把合成测试数据当成飞行证据。

入口：

```bash
cd /mnt/e/清洁无人机/drone_sim_ws
RUN_SLOW_ADAPTIVE_AB_CONFIRM=1 \
  bash scripts/run_slow_adaptive_ab_4kg.sh <campaign-id>
```

运行器对 `nominal` 和 `payload_10g_unmodeled` 各执行三对 `OFF → ON`，总计
12 次飞行。每次都固定 `GZ_RANDOM_SEED=4027`，使用独立的
`/tmp/my_drone_px4_work.*`，并执行 90 秒伸直、8 秒保持、120 秒回收。世界位置
保持固定为 P-only（`PX4_TRUTH_HOLD_XY_D=0`）；位置附加反馈、旧静态 COM
前馈、预测力矩前馈和扰动观测器均关闭，唯一实验变量是慢速自适应开关。
运行器在第一个 case 前强制 `colcon build --symlink-install` 并重新 source 当前
工作空间，避免把旧 install 中的控制器当成最新代码。

10 g 组使用临时 URDF 把质量、惯量、视觉和碰撞合并到 `gripper_link`，所以负载
真实进入 Gazebo 刚体动力学；Base 1 只读估计器仍固定读取无负载 4 kg URDF，且
`payload_mass=0`。这组因此测试的是未建模负载，不是把同一负载同时喂给补偿器。

每个 ON 日志必须证明：自适应 eligibility 成立、完成 warm-up、输出有限且非零、
不超过力/力矩限幅；日志判定器还记录相邻输出最大跳变量、非自然归零次数，以及
分配受限时的即时 back-calculation 与 feasibility continuity 是否被实际触发。
判定器会核对 back-calculation 后的内部状态是否等于实际交付的自适应分量；
没有分配受限事件时记为 `NOT_EXERCISED`，不会伪称已经覆盖。

自动接受条件：

- 12 次运行都通过 0.05 m 水平、0.05 m 高度、1° 倾角、零电机饱和、无
  failsafe 和完整伸出/回收门；
- OFF 输出必须为零，ON 必须是真实 eligibility 后的非零输出，杜绝退化为同一控制；
- nominal 三对均值不得发生超过 5% 的相对退化（另有日志量化精度余量）；
- 未建模 10 g 负载组三对均值的归一化水平/高度/倾角综合量至少改善 20%，且任一
  单项不得退化；
- 12 个 PX4 工作目录必须唯一，证明每次都是 fresh PX4。

机器可读的未运行状态为 `analysis/slow_adaptive_ab_4kg_NOT_RUN.json`。真实运行后，
每个 campaign 目录会保存 manifest、12 组 flight/backend/reallocator/PX4 日志、
PX4 工作目录证据和最终 `campaign_result.json`；只有该文件为 `ACCEPTED` 才能把
慢速自适应从候选提升为默认功能。
