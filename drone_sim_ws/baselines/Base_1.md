# Base 1：4 kg 稳定悬停与 WASD 飞行基线

冻结日期：2026-08-12

Git 恢复标签：`base-1`

## 定义

“Base 1”专指当前这套已经获得人工认可的 4 kg 飞行版本。以后提到“以 Base 1 为基础”“回到 Base 1”或“对比 Base 1”，均以 Git 标签 `base-1` 所指向的完整提交为准。

## 必须保持的能力

- 从地面起飞；
- PX4 稳定悬停；
- WASD 锁存式速度控制；
- 上升、下降、前、后、左、右速度控制；
- 偏航与正常降落；
- SO101 机械臂收回姿态和键盘控制入口；
- 补偿关闭时，机械臂控制节点不得改变飞机的悬停和 WASD 语义。

## 默认状态

- 飞行质量配置：4 kg；
- 飞行控制：现有 PX4 级联反馈控制；
- WASD：当前速度目标锁存逻辑；
- `ARM_FEEDFORWARD_ENABLED=false`；
- `ARM_TORQUE_FEEDFORWARD_ENABLED=false`；
- `ARM_STATIC_COM_FEEDFORWARD_GAIN=0`；
- `ARM_DISTURBANCE_OBSERVER_ENABLED=false`；
- `-ArmCompensationTest` 仅为显式实验入口，不属于 Base 1 默认启动方式。

## 启动规则

正常启动时不要传入 `-ArmCompensationTest`。机械臂补偿实验必须使用单独运行记录，不得覆盖 Base 1 的默认参数。

## 非回归门

后续任何机械臂、补偿器、轨迹或分配器修改，都必须与 Base 1 做对照。至少验证：

1. 机械臂关闭时，起飞、悬停、WASD、偏航和降落不退化；
2. 机械臂节点启动但静止时，飞行行为不退化；
3. 补偿数据超时或被关闭后，补偿平滑归零并回到 Base 1 飞行闭环；
4. 不允许用放宽安全门或隐藏电机饱和的方式宣称通过。

## 冻结时验证

- ROS 2/Python 静态回归：`110 passed, 3 warnings`；
- 警告均为上游 protobuf 弃用警告；
- 稳定飞行核心继承自已验收的 `baseline-4kg-wasd-hover-20260812`；
- 本次新增机械臂键盘入口和补偿实验入口均不改变默认悬停配置。

## 恢复方法

只查看或建立恢复分支：

```bash
git switch -c codex/restore-base-1 base-1
```

不要在含有未保存用户改动的工作树中直接执行强制切换或硬重置。

## 相关历史保护点

- `baseline-4kg-wasd-hover-20260812`：Base 1 的稳定飞行核心前置版本；
- `baseline-7p735-flyable`：7.735 kg 正式质量历史回退点，继续保留且不得覆盖。
