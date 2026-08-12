#!/usr/bin/env python3
"""Build and validate the auditable 1..8 motor physical-freeze table.

The cylinder axis is geometric and signless.  The report deliberately labels
the stored ray as the CAD propeller-side ray; it must never be interpreted as
the positive thrust direction until blade handedness or a signed bench test is
available.
"""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path


def fmt(values: list[float], digits: int = 6) -> str:
    return "[" + ", ".join(f"{float(value):.{digits}f}" for value in values) + "]"


def build(workspace: Path, output: Path) -> None:
    analysis = workspace / "analysis" / "cad_direct"
    status_path = analysis / "motor_physical_freeze_status.json"
    axes_path = analysis / "motor_axis_evidence.json"
    configurations_path = analysis / "propeller_configuration_evidence.json"
    metadata_path = analysis / "propeller_metadata_evidence.json"
    pitch_path = analysis / "propeller_pitch_geometry_evidence.json"
    formal_config_path = (
        workspace
        / "src"
        / "drone_arm_sim"
        / "config"
        / "my_drone_v3_cad_7p735_flight.json"
    )
    status = json.loads(status_path.read_text(encoding="utf-8"))
    axes = json.loads(axes_path.read_text(encoding="utf-8"))
    configurations = json.loads(configurations_path.read_text(encoding="utf-8"))
    metadata = json.loads(metadata_path.read_text(encoding="utf-8"))
    pitch = json.loads(pitch_path.read_text(encoding="utf-8"))
    formal_config = json.loads(formal_config_path.read_text(encoding="utf-8"))

    if status.get("schema") != 2:
        raise ValueError("motor physical-freeze status must use schema 2 semantics")
    motors = status["motors"]
    raw_by_motor = {int(row["motor"]): row for row in axes["motors"]}
    formal_by_motor = {
        int(row["motor"]): row for row in formal_config.get("rotors", [])
    }
    if [row["motor"] for row in motors] != list(range(1, 9)):
        raise ValueError("motor rows must be ordered 1..8")
    if sorted(row["esc_function"] for row in motors) != list(range(1, 9)):
        raise ValueError("PX4 ESC functions must be unique 1..8")
    if set(formal_by_motor) != set(range(1, 9)):
        raise ValueError("formal allocation config must contain motors 1..8")
    if formal_config.get("allocation_reference") != "vehicle COM in PX4 FRD":
        raise ValueError("formal allocation reference is not the vehicle COM")
    if formal_config.get("wrench_reference") != "URDF base_link origin in PX4 FRD":
        raise ValueError("formal wrench reference is not the URDF base_link origin")
    if not math.isclose(float(status["maximum_thrust_per_motor_n"]), 11.76798, abs_tol=1e-9):
        raise ValueError("maximum thrust is not the frozen 11.76798 N")
    legacy_direction_keys = {"upward_thrust_motors", "downward_thrust_motors"}
    if legacy_direction_keys.intersection(axes.get("body_frame_frozen", {})):
        raise ValueError("CAD axis evidence still mislabels geometric rays as thrust facts")
    if axes.get("body_frame_frozen", {}).get("positive_thrust_direction_status") != (
        "UNRESOLVED_FOR_ALL_MOTORS"
    ):
        raise ValueError("positive thrust direction was promoted without signed evidence")
    for row in motors:
        required_vectors = (
            "position_m",
            "cad_propeller_side_axis",
            "legacy_rotation_mount_hypothesis_axis",
            "all_up_hypothesis_axis",
        )
        if any(len(row.get(name, [])) != 3 for name in required_vectors):
            raise ValueError(f"motor {row['motor']} does not have a 3-D position/axis")
        for name in required_vectors[1:]:
            norm = math.sqrt(sum(float(value) ** 2 for value in row[name]))
            if not math.isclose(norm, 1.0, abs_tol=1e-6):
                raise ValueError(f"motor {row['motor']} {name} is not unit length")
        raw = raw_by_motor[int(row["motor"])]
        formal = formal_by_motor[int(row["motor"])]
        for actual, expected in zip(
            row["cad_propeller_side_axis"], raw["cad_propeller_side_axis_frd"]
        ):
            if not math.isclose(float(actual), float(expected), abs_tol=1e-6):
                raise ValueError(
                    f"motor {row['motor']} CAD propeller-side axis disagrees with raw evidence"
                )
        if row.get("thrust_sign_status") != (
            "UNRESOLVED_PROP_PITCH_OR_SIGNED_TEST_REQUIRED"
        ):
            raise ValueError(f"motor {row['motor']} thrust sign was promoted without evidence")
        for actual, expected in zip(row["position_m"], formal["wrench_position_m"]):
            if not math.isclose(float(actual), float(expected), abs_tol=1e-6):
                raise ValueError(
                    f"motor {row['motor']} base_link position disagrees with formal wrench point"
                )
    for motor in (4, 5, 7, 8):
        pitch_type = motors[motor - 1]["pitch_type"]
        if pitch_type != "UNRESOLVED_OPPOSITE_PITCH_REQUIRED_FOR_ALL_UP":
            raise ValueError(f"motor {motor} must remain opposite-pitch unresolved")
    if configurations.get("conclusion") != "SINGLE_CONFIGURATION_NO_HIDDEN_HANDEDNESS":
        raise ValueError("propeller configuration evidence is not frozen to one configuration")
    if configurations.get("configuration_count") != 1:
        raise ValueError("propeller source part must contain exactly one configuration")
    if len(configurations.get("instances", [])) != 8:
        raise ValueError("propeller configuration evidence must contain eight instances")
    if metadata.get("conclusion") != "NO_HANDEDNESS_METADATA_FOUND":
        raise ValueError("propeller metadata contains a handedness marker requiring review")
    hashes = pitch.get("component_reuse_evidence", {}).get(
        "unique_source_part_sha256", []
    )
    if metadata.get("source_part_sha256") not in hashes:
        raise ValueError("metadata and pitch evidence do not refer to the same propeller part")
    reuse = pitch.get("component_reuse_evidence", {})
    pitch_motors = pitch.get("motors", [])
    if reuse.get("unique_source_part_count") != 1:
        raise ValueError("pitch evidence does not freeze one common propeller source part")
    if reuse.get("all_instance_transform_determinants_positive") is not True:
        raise ValueError("at least one propeller assembly transform may reverse handedness")
    if reuse.get("mirrored_motor_instances") != []:
        raise ValueError("propeller evidence contains mirrored motor instances")
    if sorted(int(row["motor"]) for row in pitch_motors) != list(range(1, 9)):
        raise ValueError("pitch evidence must contain motors 1..8 exactly once")
    for row in pitch_motors:
        determinant = float(row.get("assembly_transform_determinant", math.nan))
        if not math.isclose(determinant, 1.0, abs_tol=1.0e-9):
            raise ValueError(
                f"motor {row['motor']} propeller transform determinant is not +1"
            )
        if row.get("mirrored_instance") is not False:
            raise ValueError(f"motor {row['motor']} propeller is marked mirrored")
        if row.get("pitch_sign_resolved") is not False:
            raise ValueError(
                f"motor {row['motor']} pitch sign was unexpectedly promoted"
            )

    lines = [
        "# 1～8 号电机物理冻结审计表",
        "",
        f"- 状态：`{status['status']}`",
        f"- 唯一几何源：`{status['authoritative_geometry_source']}`",
        "- CAD 圆柱轴来源：电机轴 2.5 mm 圆柱面，并与电机其他同轴圆柱面和桨毂圆柱面交叉验证。",
        "- SolidWorks 配置复核：源桨零件只有一个“默认”配置，八个实例全部引用它；不存在隐藏的正/反桨配置切换。",
        "- SolidWorks 元数据复核：摘要、自定义属性、配置说明、方程和特征名中均未发现 CW/CCW、正桨/反桨或左右旋标识。",
        "- 装配手性复核：八个桨实例的总装变换行列式均为 `+1`（数值误差 `1e-9` 内），没有任何镜像实例；装配变换没有生成反向螺距。",
        "- 重要定义：下表的轴是“从电机轴心指向 CAD 螺旋桨所在侧”的几何射线，不是已经确认的正推力方向。",
        "- 坐标原点严格分开：`base_link` 列供 Gazebo 施力点使用；`CAD 估算 COM` 列供 PX4 分配使用。当前 COM 来自 CAD 密度估算，尚不是实测质心。",
        "- 最大推力：每台 `11.76798 N`（用户提供 1.2 kgf，按标准重力换算）。",
        "",
        "| 电机 | CAD 总装轴心 mm | FRD/base_link 施力点 m | FRD/CAD 估算 COM 分配位置 m | CAD 桨侧轴 FRD | 安装 | 旋向 | 桨型/推力正方向 | PX4 输出 |",
        "|---:|---|---|---|---|---|---|---|---:|",
    ]
    for row in motors:
        raw = raw_by_motor[int(row["motor"])]
        formal = formal_by_motor[int(row["motor"])]
        pitch = row["pitch_type"].replace("_", " ")
        lines.append(
            f"| {row['motor']} | `{fmt(raw['assembly_axis_point_mm'])}` | "
            f"`{fmt(row['position_m'])}` | `{fmt(formal['position_m'])}` | "
            f"`{fmt(row['cad_propeller_side_axis'])}` | "
            f"{row['installation']} | {row['rotation']} | **{pitch}** | {row['esc_function']} |"
        )

    hypotheses = status["thrust_to_weight_hypotheses"]
    lines.extend([
        "",
        "## 尚未冻结的唯一关键项",
        "",
        "CAD 中八个桨实例来自同一个非镜像、几何上近似零螺距的零件，因此 CAD 不能证明任何实例是正桨或反桨。尤其是 4、5、7、8，必须取得厂家桨型标识、真实正/反桨零件，或在已知旋向/RPM 下进行带符号轴向力试验。",
        "",
        "在取得该证据以前，用户讨论中的 `0.994` 与约 `2.08` 必须继续标记为两种互斥假设，不能作为最终推重比结论。按当前 `11.76798 N` 上限和精确 CAD 轴线重算为：",
        "",
        f"- 4 kg、仅 1/2/3/6 提供向上分力：`{hypotheses['debug_4kg_four_up_only_recomputed']:.4f}`；",
        f"- 4 kg、4/5/7/8 使用相反螺距且八台均向上：`{hypotheses['debug_4kg_opposite_pitch_all_up']:.4f}`；",
        f"- 7.735 kg、仅四台向上：`{hypotheses['formal_7p735kg_four_up_only']:.4f}`；",
        f"- 7.735 kg、八台均向上：`{hypotheses['formal_7p735kg_opposite_pitch_all_up']:.4f}`。",
        "",
        "其中 `0.994`（旧计算记录为 `0.9936`）与当前四桨向上重算值 `1.0392` 不一致；这项差异本身也是未冻结输入的证据，二者都不得冒充最终物理结论。`2.08` 是八桨全向上的四舍五入假设，同样必须等 4/5/7/8 反向螺距证据后才能提升。",
        "",
        "## 闭环证据格式",
        "",
        "对 4、5、7、8 每台记录：桨零件型号/正反桨标识、观察旋向定义、测试 RPM、轴向力符号和大小、照片或试验日志。四台证据齐全后才允许把 `pitch_type` 改为确定值，并重新生成正式控制分配矩阵。",
        "",
        "证据必须先填入 `motor_thrust_evidence.template.json` 的副本，并通过 `scripts/validate_motor_thrust_evidence.py`；空模板和缺字段记录会返回非零退出码、`status=INCOMPLETE`、`promotion_allowed=false`，禁止物理提升。",
        "",
        "`MOTOR_PHYSICAL_FREEZE_REPORT_PASS`",
    ])
    output.write_text("\n".join(lines) + "\n", encoding="utf-8")


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--workspace",
        type=Path,
        default=Path(__file__).resolve().parents[1],
    )
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    output = args.output or args.workspace / "analysis" / "cad_direct" / "final_motor_evidence_table.md"
    build(args.workspace.resolve(), output.resolve())
    print(f"MOTOR_PHYSICAL_FREEZE_REPORT_PASS output={output.resolve()}")


if __name__ == "__main__":
    main()
