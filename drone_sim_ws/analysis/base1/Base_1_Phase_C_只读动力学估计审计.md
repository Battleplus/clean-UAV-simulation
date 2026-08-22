# Base 1 Phase C 只读动力学估计审计

日期：2026-08-13

范围：仅 `base-1` / 4 kg；未启动、测试、读取或调参 7.735 kg 飞行配置。所有补偿输出保持关闭。

## 实现内容

独立的 Base 1 estimator overlay 以 100 Hz 名义周期从 SO101 关节状态计算并记录。Base 1 原 launch 的 3 Hz 默认值和核心文件哈希保持不变；overlay 所有输出均重映射到 `/my_drone/base1_estimator/*`，不会覆盖原话题：

- 整机动态质心 `com_body_flu_m`；
- 整机完整 3×3 惯量 `inertia_tensor_kg_m2`；
- 机械臂连杆加减速产生的反作用力 `reaction_force_body_n`；
- 动态反作用力矩 `reaction_torque_body_nm`；
- 质心偏移产生的静态重力矩 `gravity_shift_torque_body_nm`。

状态消息明确包含：`estimator_mode=read_only`、源关节时间戳、输出时间戳、源数据年龄、100 ms 超时、有效标志和 `base_link_flu` frame。源数据超过 100 ms 时不刷新 wrench/acceleration 时间戳，使后续控制安全门能观察到真实超时。

## 算法等价性和计算余量

旧实现每次约 97.9 ms，极限约 10.2 Hz，不能直接改参数冒充 100 Hz。优化后使用：

1. URDF 固定质量、局部质心和惯量缓存；
2. 一次树遍历同时计算 link transform 与关节轴；
3. 精确几何 Jacobian 代替每关节整树差分；
4. 仅对 `J_dot` 沿当前关节速度方向做中心差分。

五次离线基准的最慢计算为 5.39 ms，对应 185.6 Hz。与旧参考算法最大绝对误差：

- 质心：0；
- 完整惯量：0；
- 反作用力：6.31e-9 N；
- 反作用力矩：1.64e-9 N·m。

证据：`base1_coupling_estimator_compute_benchmark.json`。

## 实际 ROS 节点验收

12 秒实测：

- 1185 个状态样本；
- 实测频率 99.51 Hz；
- 周期中位数 9.96 ms；
- 周期 P99 16.44 ms；
- 有效样本 1185/1185；
- 最大源数据年龄 39.2 ms；
- 质量严格为 4.0 kg；
- 动态质心、完整惯量、反作用力与力矩全部为有限值。

这是 100 Hz ROS 定时器在 WSL 调度下的实测结果，验收门为持续频率不低于 95 Hz、有效率不低于 99%、源数据不超过 100 ms。

## 当前判定

Phase C 的 100 Hz 只读 estimator overlay 已建立并通过离线等价性、计算余量和实际 ROS 发布验证。Base 1 核心 launch 默认值未修改。当前仍未把任何估计量注入电机或 PX4 控制；下一阶段才建立单一的分配前 6D wrench 合成入口及 fail-closed 安全回零逻辑。
