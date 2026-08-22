"""Plan and execute the key-6 tool-forward Cartesian arm demonstration."""

from __future__ import annotations

import argparse
from collections.abc import Callable
import math
from pathlib import Path
import sys
import time

from ament_index_python.packages import get_package_share_directory
from builtin_interfaces.msg import Duration
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from drone_arm_sim.inverse_kinematics import ARM_JOINTS
from drone_arm_sim.model_analysis import UrdfModel


JOINT_NAMES = ARM_JOINTS + ["gripper"]
# The gripper is mounted coaxially with the CAD-extracted wrist-roll drive
# horn.  Using that measured cylinder axis, rather than an idealized +X, also
# preserves the real 0.504-degree assembly inclination.
TOOL_FORWARD_AXIS_LOCAL = np.array([0.999961256, -0.000000665, 0.008802682])
TOOL_FORWARD_AXIS_LOCAL /= np.linalg.norm(TOOL_FORWARD_AXIS_LOCAL)


def _formal_urdf() -> Path:
    return (
        Path(get_package_share_directory("drone_arm_sim"))
        / "urdf" / "my_drone_v3" / "my_drone_cad_formal_dynamic.urdf"
    )


def _candidate_seeds(previous: dict[str, float]) -> list[dict[str, float]]:
    """Escape the all-zero folded-chain singularity without changing the task."""
    result = [dict(previous)]
    for sign in (-1.0, 1.0):
        for scale in (0.08, 0.16, 0.28):
            seed = dict(previous)
            seed["shoulder_lift"] += sign * scale
            seed["elbow_flex"] -= sign * 2.0 * scale
            seed["wrist_flex"] += sign * scale
            result.append(seed)
    return result


def _solve_tool_axis_ik(
    model: UrdfModel,
    target_position: np.ndarray,
    target_forward: np.ndarray,
    seed: dict[str, float],
    max_iterations: int = 1200,
    damping: float = 0.02,
) -> tuple[dict[str, float], dict[str, object]]:
    """Solve XYZ exactly and keep tool-axis change softly minimized.

    The CAD chain has five joints, but wrist roll is coaxial with the tool and
    is deliberately held fixed to avoid reaction torque.  The remaining four
    joints cannot in general satisfy XYZ plus two exact direction constraints.
    The requested Cartesian contract is therefore the straight TCP line;
    orientation is regularized and reported, not falsely declared exact.
    """
    positions = dict(seed)
    position_error = math.inf
    direction_error = math.inf
    for iteration in range(max_iterations):
        current, active = model.forward_kinematics("gripper_link", positions)
        names = [item[0] for item in active]
        jacobian = model.jacobian("gripper_link", positions)
        current_forward = current[:3, :3] @ TOOL_FORWARD_AXIS_LOCAL
        delta_position = target_position - current[:3, 3]
        delta_direction = np.cross(current_forward, target_forward)
        position_error = float(np.linalg.norm(delta_position))
        direction_error = float(np.linalg.norm(delta_direction))
        if position_error < 1.0e-4:
            break
        # Rotation about the tool axis does not alter its direction.  Remove
        # that unobservable component so the 5-DoF SO101 is not asked to solve
        # an impossible arbitrary 6D pose.
        projector = np.eye(3) - np.outer(current_forward, current_forward)
        task_jacobian = np.vstack((jacobian[:3], projector @ jacobian[3:]))
        axis_weight = 0.04
        error = np.concatenate((delta_position, axis_weight * delta_direction))
        weighted_jacobian = task_jacobian.copy()
        weighted_jacobian[3:] *= axis_weight
        normal = (
            weighted_jacobian @ weighted_jacobian.T
            + damping * damping * np.eye(6)
        )
        increment = weighted_jacobian.T @ np.linalg.solve(normal, error)
        norm = float(np.linalg.norm(increment))
        if norm > 0.12:
            increment *= 0.12 / norm
        for name, delta in zip(names, increment):
            if name == "wrist_roll":
                # Rotation about the measured tool axis is unobservable in
                # this task and needlessly injects a large reaction torque.
                continue
            lower, upper = model.joint_limits(name)
            positions[name] = float(np.clip(positions[name] + delta, lower, upper))
    return positions, {
        "converged": position_error < 1.0e-4,
        "iterations": iteration + 1,
        "position_error_m": position_error,
        "tool_axis_error_rad": math.asin(min(1.0, direction_error)),
    }


def plan_tool_forward_path(
    model: UrdfModel,
    start_positions: dict[str, float],
    distance_m: float = 0.12,
    step_m: float = 0.005,
    joint_margin_rad: float = math.radians(5.0),
) -> tuple[list[dict[str, float]], dict[str, object]]:
    """Sample a fixed-orientation straight line along gripper-link local +X."""
    if distance_m <= 0.0 or step_m <= 0.0:
        raise ValueError("distance and step must be positive")
    start = {name: float(start_positions.get(name, 0.0)) for name in ARM_JOINTS}
    start_transform, _ = model.forward_kinematics("gripper_link", start)
    forward = start_transform[:3, :3] @ TOOL_FORWARD_AXIS_LOCAL
    forward /= np.linalg.norm(forward)
    distances = list(np.arange(step_m, distance_m + 0.5 * step_m, step_m))
    if not distances or distances[-1] < distance_m - 1.0e-9:
        distances.append(distance_m)

    path = [dict(start)]
    statuses: list[dict[str, object]] = []
    previous = dict(start)
    for distance in distances:
        target = start_transform.copy()
        target[:3, 3] = start_transform[:3, 3] + float(distance) * forward
        candidates = []
        for seed in _candidate_seeds(previous):
            solution, status = _solve_tool_axis_ik(
                model, target[:3, 3], forward, seed,
            )
            if not bool(status["converged"]):
                continue
            inside_margin = True
            for name in ARM_JOINTS:
                lower, upper = model.joint_limits(name)
                if not lower + joint_margin_rad <= solution[name] <= upper - joint_margin_rad:
                    inside_margin = False
                    break
            if inside_margin:
                change = sum((solution[name] - previous[name]) ** 2 for name in ARM_JOINTS)
                candidates.append((change, solution, status))
        if not candidates:
            raise RuntimeError(
                f"Cartesian extension is unreachable at {distance:.3f} m "
                f"with a {math.degrees(joint_margin_rad):.1f} deg joint margin"
            )
        _, previous, status = min(candidates, key=lambda item: item[0])
        path.append(dict(previous))
        statuses.append({"distance_m": float(distance), **status})

    joint_steps = [
        {
            name: float(path[index][name] - path[index - 1][name])
            for name in ARM_JOINTS
        }
        for index in range(1, len(path))
    ]
    max_step_index = max(
        range(len(joint_steps)),
        key=lambda index: max(abs(value) for value in joint_steps[index].values()),
    )
    evidence = {
        "end_link": "gripper_link",
        "tool_forward_axis_local": TOOL_FORWARD_AXIS_LOCAL.tolist(),
        "tool_forward_axis_base_at_start": forward.tolist(),
        "start_position_m": start_transform[:3, 3].tolist(),
        "requested_distance_m": float(distance_m),
        "step_m": float(step_m),
        "sample_count_outward": len(path),
        "maximum_joint_step_rad": max(
            abs(value) for value in joint_steps[max_step_index].values()
        ),
        "maximum_joint_step_ending_distance_m": float(distances[max_step_index]),
        "maximum_joint_step_by_joint_rad": joint_steps[max_step_index],
        "ik_status": statuses,
    }
    return path, evidence


def _trajectory_points(
    outward: list[dict[str, float]], gripper: float,
    out_duration_s: float, hold_s: float,
) -> list[JointTrajectoryPoint]:
    joint_rows = [np.array([pose[name] for name in ARM_JOINTS] + [gripper]) for pose in outward]
    # Reverse the exact solved joint path.  The duplicated endpoint is the
    # explicit extended hold and cannot select a different IK branch.
    rows = joint_rows + [joint_rows[-1].copy()] + list(reversed(joint_rows[:-1]))
    count = len(joint_rows)
    reverse_step_s = out_duration_s / max(1, count - 1)
    times = np.concatenate((
        np.linspace(0.0, out_duration_s, count),
        [out_duration_s + hold_s],
        np.linspace(
            out_duration_s + hold_s + reverse_step_s,
            2.0 * out_duration_s + hold_s,
            count - 1,
        ),
    ))
    points = []
    for index, (row, elapsed) in enumerate(zip(rows, times)):
        point = JointTrajectoryPoint()
        point.positions = row.tolist()
        if index in (0, count - 1, count, len(rows) - 1):
            point.velocities = [0.0] * len(JOINT_NAMES)
            point.accelerations = [0.0] * len(JOINT_NAMES)
        else:
            dt = max(1.0e-6, times[index + 1] - times[index - 1])
            point.velocities = ((rows[index + 1] - rows[index - 1]) / dt).tolist()
            dt_before = max(1.0e-6, times[index] - times[index - 1])
            dt_after = max(1.0e-6, times[index + 1] - times[index])
            velocity_before = (rows[index] - rows[index - 1]) / dt_before
            velocity_after = (rows[index + 1] - rows[index]) / dt_after
            point.accelerations = (
                2.0 * (velocity_after - velocity_before) / (dt_before + dt_after)
            ).tolist()
        seconds = int(elapsed)
        nanoseconds = int(round((elapsed - seconds) * 1_000_000_000))
        if nanoseconds == 1_000_000_000:
            seconds += 1
            nanoseconds = 0
        point.time_from_start = Duration(sec=seconds, nanosec=nanoseconds)
        points.append(point)
    return points


class CartesianDemoNode(Node):
    def __init__(self) -> None:
        super().__init__("cartesian_arm_demo")
        self.positions: dict[str, float] = {}
        self.last_state_monotonic = 0.0
        self.trajectory_pub = self.create_publisher(
            JointTrajectory, "/arm_controller/joint_trajectory", 10
        )
        self.motion_pub = self.create_publisher(Bool, "/my_drone/arm_motion_active", 10)
        self.create_subscription(JointState, "/joint_states", self._state_cb, 10)

    def _state_cb(self, message: JointState) -> None:
        self.positions.update(zip(message.name, message.position))
        self.last_state_monotonic = time.monotonic()

    def publish_motion(self, active: bool) -> None:
        self.motion_pub.publish(Bool(data=active))


def wait_for_arm_ready(node: CartesianDemoNode, timeout_s: float = 10.0) -> None:
    deadline = time.monotonic() + timeout_s
    while node.trajectory_pub.get_subscription_count() == 0 or any(
        name not in node.positions for name in JOINT_NAMES
    ):
        if time.monotonic() >= deadline:
            raise RuntimeError("arm controller or complete joint state is unavailable")
        rclpy.spin_once(node, timeout_sec=0.1)


def execute_cartesian_cycle(
    node: CartesianDemoNode,
    model: UrdfModel,
    *,
    distance: float,
    step: float,
    duration: float,
    hold: float,
    joint_margin_rad: float,
    cycle_label: str = "1",
) -> dict[str, float | object]:
    """Execute one measured out-and-back cycle without recreating the ROS node."""
    wait_for_arm_ready(node)
    path, evidence = plan_tool_forward_path(
        model, node.positions, distance, step, joint_margin_rad,
    )
    message = JointTrajectory()
    message.joint_names = JOINT_NAMES
    message.points = _trajectory_points(
        path, float(node.positions["gripper"]), duration, hold
    )
    total_duration = 2.0 * duration + hold
    start_monotonic = time.monotonic()
    node.publish_motion(True)
    node.trajectory_pub.publish(message)
    node.get_logger().info(
        "CARTESIAN_DEMO_BEGIN "
        f"cycle={cycle_label} monotonic={start_monotonic:.6f} "
        f"distance={distance:.3f}m step={step:.3f}m "
        f"axis_base={evidence['tool_forward_axis_base_at_start']}"
    )
    finished = start_monotonic + total_duration
    last_heartbeat = 0.0
    start_position = np.asarray(evidence["start_position_m"], dtype=float)
    forward_axis = np.asarray(evidence["tool_forward_axis_base_at_start"], dtype=float)
    max_progress = -math.inf
    max_cross_track = 0.0
    while time.monotonic() < finished:
        rclpy.spin_once(node, timeout_sec=0.05)
        if all(name in node.positions for name in ARM_JOINTS):
            current_transform, _ = model.forward_kinematics(
                "gripper_link", node.positions
            )
            displacement = current_transform[:3, 3] - start_position
            progress = float(displacement @ forward_axis)
            cross_track = float(np.linalg.norm(np.cross(displacement, forward_axis)))
            max_progress = max(max_progress, progress)
            max_cross_track = max(max_cross_track, cross_track)
        if time.monotonic() - last_heartbeat >= 0.25:
            node.publish_motion(True)
            last_heartbeat = time.monotonic()

    settle_deadline = time.monotonic() + 10.0
    return_stable_since = None
    max_return_error = math.inf
    while time.monotonic() < settle_deadline:
        rclpy.spin_once(node, timeout_sec=0.05)
        state_fresh = time.monotonic() - node.last_state_monotonic < 0.5
        if state_fresh and all(name in node.positions for name in ARM_JOINTS):
            max_return_error = max(
                abs(float(node.positions[name]) - float(path[0][name]))
                for name in ARM_JOINTS
            )
            if max_return_error <= 0.08:
                if return_stable_since is None:
                    return_stable_since = time.monotonic()
                elif time.monotonic() - return_stable_since >= 0.5:
                    break
            else:
                return_stable_since = None
        if time.monotonic() - last_heartbeat >= 0.25:
            node.publish_motion(True)
            last_heartbeat = time.monotonic()

    completed_monotonic = time.monotonic()
    node.get_logger().info(
        "CARTESIAN_DEMO_COMPLETE "
        f"cycle={cycle_label} monotonic={completed_monotonic:.6f} "
        f"return_error={max_return_error:.6f}rad "
        f"max_progress={max_progress:.6f}m max_cross_track={max_cross_track:.6f}m"
    )
    if max_progress < distance - max(step, 0.01):
        raise RuntimeError(
            f"Cartesian demo did not extend far enough: {max_progress:.3f} m"
        )
    if return_stable_since is None or max_return_error > 0.08:
        raise RuntimeError(
            f"Cartesian demo did not return to start: {max_return_error:.3f} rad"
        )
    return {
        "started_monotonic": start_monotonic,
        "completed_monotonic": completed_monotonic,
        "max_return_error": max_return_error,
        "max_progress": max_progress,
        "max_cross_track": max_cross_track,
        "evidence": evidence,
    }


def execute_cartesian_cycle_gated(
    node: CartesianDemoNode,
    model: UrdfModel,
    *,
    distance: float,
    step: float,
    duration: float,
    hold: float,
    joint_margin_rad: float,
    waypoint_gate: Callable[[], None],
    cycle_label: str = "1",
) -> dict[str, float | object]:
    """Execute the same Cartesian path one waypoint at a time.

    This is reserved for the formal low-thrust-margin aircraft.  Each solved
    5 mm Cartesian waypoint is completed and the caller's measured-flight
    stability gate must pass before the next waypoint can begin.  The exact
    solved path is reversed, so gating changes timing, not path geometry.
    """
    wait_for_arm_ready(node)
    path, evidence = plan_tool_forward_path(
        model, node.positions, distance, step, joint_margin_rad,
    )
    gripper = float(node.positions["gripper"])
    outward_rows = [
        np.array([pose[name] for name in ARM_JOINTS] + [gripper], dtype=float)
        for pose in path
    ]
    rows = outward_rows + list(reversed(outward_rows[:-1]))
    outward_segments = max(1, len(outward_rows) - 1)
    segment_duration_s = duration / outward_segments
    start_monotonic = time.monotonic()
    start_position = np.asarray(evidence["start_position_m"], dtype=float)
    forward_axis = np.asarray(evidence["tool_forward_axis_base_at_start"], dtype=float)
    max_progress = -math.inf
    max_cross_track = 0.0
    node.get_logger().info(
        "CARTESIAN_DEMO_BEGIN "
        f"cycle={cycle_label} monotonic={start_monotonic:.6f} "
        f"distance={distance:.3f}m step={step:.3f}m "
        f"axis_base={evidence['tool_forward_axis_base_at_start']} "
        f"mode=flight_gated segment_duration={segment_duration_s:.3f}s"
    )

    for index in range(1, len(rows)):
        message = JointTrajectory()
        message.joint_names = JOINT_NAMES
        start_point = JointTrajectoryPoint()
        start_point.positions = rows[index - 1].tolist()
        start_point.velocities = [0.0] * len(JOINT_NAMES)
        start_point.time_from_start = Duration(sec=0, nanosec=0)
        target_point = JointTrajectoryPoint()
        target_point.positions = rows[index].tolist()
        target_point.velocities = [0.0] * len(JOINT_NAMES)
        seconds = int(segment_duration_s)
        nanoseconds = int(round((segment_duration_s - seconds) * 1_000_000_000))
        target_point.time_from_start = Duration(sec=seconds, nanosec=nanoseconds)
        message.points = [start_point, target_point]
        node.publish_motion(True)
        node.trajectory_pub.publish(message)
        deadline = time.monotonic() + segment_duration_s
        last_heartbeat = 0.0
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
            if all(name in node.positions for name in ARM_JOINTS):
                current_transform, _ = model.forward_kinematics(
                    "gripper_link", node.positions
                )
                displacement = current_transform[:3, 3] - start_position
                max_progress = max(max_progress, float(displacement @ forward_axis))
                max_cross_track = max(
                    max_cross_track,
                    float(np.linalg.norm(np.cross(displacement, forward_axis))),
                )
            if time.monotonic() - last_heartbeat >= 0.25:
                node.publish_motion(True)
                last_heartbeat = time.monotonic()
        node.publish_motion(False)
        waypoint_gate()
        if index == len(outward_rows) - 1 and hold > 0.0:
            hold_deadline = time.monotonic() + hold
            while time.monotonic() < hold_deadline:
                rclpy.spin_once(node, timeout_sec=0.05)
                node.publish_motion(False)

    max_return_error = max(
        abs(float(node.positions[name]) - float(path[0][name]))
        for name in ARM_JOINTS
    )
    completed_monotonic = time.monotonic()
    node.get_logger().info(
        "CARTESIAN_DEMO_COMPLETE "
        f"cycle={cycle_label} monotonic={completed_monotonic:.6f} "
        f"return_error={max_return_error:.6f}rad "
        f"max_progress={max_progress:.6f}m max_cross_track={max_cross_track:.6f}m "
        "mode=flight_gated"
    )
    if max_progress < distance - max(step, 0.01):
        raise RuntimeError(
            f"Cartesian demo did not extend far enough: {max_progress:.3f} m"
        )
    if max_return_error > 0.08:
        raise RuntimeError(
            f"Cartesian demo did not return to start: {max_return_error:.3f} rad"
        )
    return {
        "started_monotonic": start_monotonic,
        "completed_monotonic": completed_monotonic,
        "max_return_error": max_return_error,
        "max_progress": max_progress,
        "max_cross_track": max_cross_track,
        "evidence": evidence,
    }


def main(args=None) -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--urdf", type=Path, default=_formal_urdf())
    parser.add_argument("--distance", type=float, default=0.12)
    parser.add_argument("--step", type=float, default=0.005)
    parser.add_argument("--duration", type=float, default=8.0, help="seconds each way")
    parser.add_argument("--hold", type=float, default=3.0)
    parser.add_argument("--joint-margin-deg", type=float, default=5.0)
    parser.add_argument("--plan-only", action="store_true")
    parsed = parser.parse_args(args)
    if parsed.duration <= 0.0 or parsed.hold < 0.0:
        parser.error("duration must be positive and hold cannot be negative")

    model = UrdfModel(parsed.urdf)
    if parsed.plan_only:
        start = {name: 0.0 for name in JOINT_NAMES}
        path, evidence = plan_tool_forward_path(
            model, start, parsed.distance, parsed.step,
            math.radians(parsed.joint_margin_deg),
        )
        print(evidence)
        print("final_joints_rad", path[-1])
        return

    rclpy.init()
    node = CartesianDemoNode()
    try:
        deadline = time.monotonic() + 10.0
        while node.trajectory_pub.get_subscription_count() == 0 or any(
            name not in node.positions for name in JOINT_NAMES
        ):
            if time.monotonic() >= deadline:
                raise RuntimeError("arm controller or complete joint state is unavailable")
            rclpy.spin_once(node, timeout_sec=0.1)
        path, evidence = plan_tool_forward_path(
            model, node.positions, parsed.distance, parsed.step,
            math.radians(parsed.joint_margin_deg),
        )
        message = JointTrajectory()
        message.joint_names = JOINT_NAMES
        message.points = _trajectory_points(
            path, float(node.positions["gripper"]), parsed.duration, parsed.hold
        )
        total_duration = 2.0 * parsed.duration + parsed.hold
        node.publish_motion(True)
        node.trajectory_pub.publish(message)
        node.get_logger().info(
            "CARTESIAN_DEMO_BEGIN "
            f"distance={parsed.distance:.3f}m step={parsed.step:.3f}m "
            f"axis_base={evidence['tool_forward_axis_base_at_start']}"
        )
        finished = time.monotonic() + total_duration
        last_heartbeat = 0.0
        start_position = np.asarray(evidence["start_position_m"], dtype=float)
        forward_axis = np.asarray(evidence["tool_forward_axis_base_at_start"], dtype=float)
        max_progress = -math.inf
        max_cross_track = 0.0
        while time.monotonic() < finished:
            rclpy.spin_once(node, timeout_sec=0.05)
            if all(name in node.positions for name in ARM_JOINTS):
                current_transform, _ = model.forward_kinematics(
                    "gripper_link", node.positions
                )
                displacement = current_transform[:3, 3] - start_position
                progress = float(displacement @ forward_axis)
                cross_track = float(np.linalg.norm(np.cross(displacement, forward_axis)))
                max_progress = max(max_progress, progress)
                max_cross_track = max(max_cross_track, cross_track)
            if time.monotonic() - last_heartbeat >= 0.25:
                node.publish_motion(True)
                last_heartbeat = time.monotonic()
        # The trajectory timestamp is the command endpoint, not proof that a
        # gravity-loaded simulated joint has already settled.  Continue
        # observing fresh joint states for a bounded interval and accept only
        # after the real arm has remained within tolerance for 0.5 s.
        settle_deadline = time.monotonic() + 10.0
        return_stable_since = None
        max_return_error = math.inf
        while time.monotonic() < settle_deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
            state_fresh = time.monotonic() - node.last_state_monotonic < 0.5
            if state_fresh and all(name in node.positions for name in ARM_JOINTS):
                max_return_error = max(
                    abs(float(node.positions[name]) - float(path[0][name]))
                    for name in ARM_JOINTS
                )
                if max_return_error <= 0.08:
                    if return_stable_since is None:
                        return_stable_since = time.monotonic()
                    elif time.monotonic() - return_stable_since >= 0.5:
                        break
                else:
                    return_stable_since = None
            if time.monotonic() - last_heartbeat >= 0.25:
                node.publish_motion(True)
                last_heartbeat = time.monotonic()
        node.get_logger().info(
            f"CARTESIAN_DEMO_COMPLETE return_error={max_return_error:.6f}rad "
            f"max_progress={max_progress:.6f}m max_cross_track={max_cross_track:.6f}m"
        )
        if max_progress < parsed.distance - max(parsed.step, 0.01):
            raise RuntimeError(
                f"Cartesian demo did not extend far enough: {max_progress:.3f} m"
            )
        if return_stable_since is None or max_return_error > 0.08:
            raise RuntimeError(f"Cartesian demo did not return to start: {max_return_error:.3f} rad")
    except RuntimeError as error:
        node.get_logger().error(str(error))
        sys.exit(1)
    finally:
        node.publish_motion(False)
        rclpy.spin_once(node, timeout_sec=0.1)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
