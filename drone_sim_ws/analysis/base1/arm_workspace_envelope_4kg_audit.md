# 4 kg Base 1 机械臂安全工作空间审计

## 结论边界

`arm_workspace_envelope_4kg.json` 是离散静态包线，不是连续空间或动态飞行安全证明。
它用于在轨迹规划前排除明显不可行姿态；每条实际轨迹仍必须经过连续轨迹预演，最后还要通过
PX4/Gazebo 动态与接触验证。

当前 4 kg 配置仍采用全电机向上推力的 debug 假设，螺旋桨真实带符号推力方向尚未冻结；
舵机 `effort_nm` 也来自运动参考文件而不是台架实测。因此报告中的
`formal_physical_release_ready` 固定为 `false`。

## 旧 schema 1 的缺口

- “覆盖完整”只分别统计八个水平方向和上下方向，未证明不同伸距或夹爪状态。
- 锚点没有被重新进行高度分类，输出长期显示 `unclassified`。
- 只使用 8 个 URDF box；8 个旋翼没有 collision geometry，机械臂进入桨盘的风险没有显式代理。
- 只检查飞行电机分配，没有检查机械臂舵机是否保留静态承载余量。
- 从已接受锚点自动学习全局碰撞排除，会把仅在折叠姿态出现的粗箱重叠永久忽略。
- 输入文件没有哈希，无法证明一份报告对应哪一版 URDF、运动参考和飞行配置。

## schema 2 的改进

### 覆盖矩阵

标准网格仍为：

```text
shoulder_pan   7
shoulder_lift  7
elbow_flex     7
wrist_flex     5
wrist_roll     5
gripper        3
总计          25,725 poses
```

报告现在同时保存并验收：

- 八个水平扇区以及四个对角扇区；
- 相对收回末端高度的 `up / level / down`；
- `near <= 0.15 m`、`middle <= 0.25 m`、`far > 0.25 m`；
- 夹爪 `closed / middle / open`；
- 方向×高度、方向×伸距、方向×夹爪三张交叉矩阵；
- 每个水平方向至少有两个不同伸距；
- 每个水平方向均有三种夹爪状态。

“完整”只表示上述离散类别全部有允许样本，不表示每个方向都存在任意远伸距。例如当前
左、后、左后和右后方向没有 `far` 允许样本，这会在方向×伸距矩阵中明确显示，而不会被
总布尔值掩盖。

### 碰撞代理

- 保留 8 个 URDF link box。
- 从 8 份正式 rotor STL 自动计算旋翼扫掠体；CAD 桨半径约 `0.064 m`，再加入
  `0.005 m` 安全余量，代理半径约 `0.069 m`。
- 固定机体代理之间的重叠不参与随机械臂姿态变化的自碰判断。
- 全局只排除两个在所有已知姿态中永久嵌套的安装粗箱。
- 折叠姿态的 `upper_arm_link / wrist_link` 粗箱重叠只允许作为该锚点专属例外，
  不再泄漏到整个工作空间。
- 旋翼扫掠体绝不参与锚点校准排除。

OBB 仍是保守代理，不替代三角网格连续碰撞或 Gazebo contact。报告因此保留
`mesh_contact_validation_required=true`。

### 动力与执行器静态判据

每个姿态必须同时满足：

- 无碰撞代理命中；
- 重力矩不超过 `1.35 N·m`；
- 6×8 分配残差不超过 `1e-6`；
- 单电机补偿变化不超过 `1.60 N`，且剩余 overlay 余量至少 `0.05 N`；
- 物理电机上下界余量至少 `0.25 N`；
- 通过 CAD 质量模型和各连杆线速度 Jacobian 计算的关节静态重力负载，
  不超过文档 effort 的 90%，即保留 10% 舵机余量。

这仍未检查速度、加速度、jerk、动态反作用和轨迹段中间的连续碰撞；这些属于轨迹预演层。

## 标准网格复核结果

最终源码的完整网格复核结果：

```text
sample_count                         25,725
flight_allowed_count                  8,255
flight_allowed_fraction              0.3208940719
minimum_allowed_joint_effort_margin   0.8706387105 N·m
```

允许样本类别计数：

| 类别 | 允许样本数 |
|---|---:|
| front | 1,010 |
| front_left | 1,476 |
| left | 1,060 |
| rear_left | 448 |
| rear | 348 |
| rear_right | 315 |
| right | 2,042 |
| front_right | 788 |
| up | 113 |
| level | 419 |
| down | 7,723 |
| near | 4,958 |
| middle | 2,801 |
| far | 496 |
| gripper closed | 3,400 |
| gripper middle | 3,460 |
| gripper open | 1,395 |

拒绝原因可重叠计数：

```text
collision_proxy          16,087
allocation_residual       3,491
overlay_delta_headroom    3,953
gravity_torque_limit      1,133
```

当前网格没有触发 `joint_effort_reserve` 拒绝；最差允许姿态仍有约 `0.871 N·m`
静态舵机余量。此结果只对当前 4 kg 缩放质量模型和运动参考中的 effort 数值成立。

## 可复现命令

在默认 WSL 中：

```bash
cd '/mnt/e/清洁无人机/drone_sim_ws'
source /opt/ros/jazzy/setup.bash
source install/setup.bash
python3 scripts/scan_arm_workspace_4kg.py
python3 -m pytest -q src/drone_arm_sim/test/test_workspace_envelope.py
```

快速 smoke 模式会写入独立的 `arm_workspace_envelope_4kg_quick.json`，不会覆盖标准报告：

```bash
python3 scripts/scan_arm_workspace_4kg.py --quick
```

smoke 通过只证明扫描链路和两个已验证锚点可计算，不证明完整类别覆盖。
