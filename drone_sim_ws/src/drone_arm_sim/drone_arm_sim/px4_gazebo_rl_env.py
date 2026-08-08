"""Gym-style client for the live PX4/Gazebo aerial-manipulator bridge.
Unlike :mod:`drone_arm_sim.rl_env`, observations in this class come from the
running PX4 SITL and Gazebo model.  Flight actions remain high-level and PX4
keeps attitude/position authority; the six arm actions go to the independent
joint trajectory controller.
"""

from __future__ import annotations

import threading
import time

import numpy as np

try:
    import rclpy
    from rclpy.context import Context
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node
    from std_msgs.msg import Float32MultiArray
except ModuleNotFoundError:
    rclpy = None
    Context = SingleThreadedExecutor = Node = Float32MultiArray = None


class Px4GazeboArmEnv:
    """Synchronous step interface over the live ROS 2 PX4/Gazebo topics.

    Action layout is ``[forward, right, up, yaw_rate, joint_velocity×6,
    episode_command]`` in ``[-1, 1]``.  ``episode_command > 0.5`` requests
    takeoff and ``< -0.5`` requests PX4 LAND.  Observation layout matches the
    25-element offline environment.
    """

    observation_size = 25
    action_size = 11

    def __init__(self, observation_timeout_s: float = 5.0) -> None:
        if rclpy is None:
            raise RuntimeError("ROS 2 Python packages are required")
        self.context = Context()
        rclpy.init(context=self.context)
        self.node = Node("my_drone_px4_gazebo_rl_env", context=self.context)
        self.executor = SingleThreadedExecutor(context=self.context)
        self.executor.add_node(self.node)
        self.publisher = self.node.create_publisher(
            Float32MultiArray, "/my_drone/rl_action", 10
        )
        self.node.create_subscription(
            Float32MultiArray,
            "/my_drone/rl_observation",
            self._observation_cb,
            10,
        )
        self.node.create_subscription(
            Float32MultiArray, "/my_drone/rl_status", self._status_cb, 10
        )
        self.observation_timeout_s = float(observation_timeout_s)
        self._condition = threading.Condition()
        self._observation: np.ndarray | None = None
        self._status: np.ndarray | None = None
        self._sequence = 0
        self._closed = False
        self._origin = np.zeros(3)
        self._thread = threading.Thread(target=self._spin, daemon=True)
        self._thread.start()

    def _spin(self) -> None:
        while not self._closed and self.context.ok():
            self.executor.spin_once(timeout_sec=0.05)

    def _observation_cb(self, message) -> None:
        values = np.asarray(message.data, dtype=np.float32)
        if values.shape != (self.observation_size,) or not np.all(np.isfinite(values)):
            return
        with self._condition:
            self._observation = values.copy()
            self._sequence += 1
            self._condition.notify_all()

    def _status_cb(self, message) -> None:
        values = np.asarray(message.data, dtype=np.float32)
        if values.shape == (4,) and np.all(np.isfinite(values)):
            with self._condition:
                self._status = values.copy()
                self._condition.notify_all()

    def _wait_observation(self, after_sequence: int = -1) -> np.ndarray:
        deadline = time.monotonic() + self.observation_timeout_s
        with self._condition:
            while self._observation is None or self._sequence <= after_sequence:
                remaining = deadline - time.monotonic()
                if remaining <= 0.0:
                    raise TimeoutError("live PX4/Gazebo observation timed out")
                self._condition.wait(remaining)
            return self._observation.copy()

    def _publish_action(self, action: np.ndarray) -> None:
        values = np.asarray(action, dtype=float)
        if values.shape != (self.action_size,) or not np.all(np.isfinite(values)):
            raise ValueError(f"action must be {self.action_size} finite values")
        values = np.clip(values, -1.0, 1.0)
        self.publisher.publish(Float32MultiArray(data=values.astype(np.float32).tolist()))

    def reset(self, *, seed=None, options: dict | None = None):
        del seed
        observation = self._wait_observation()
        self._origin = observation[:3].astype(float)
        options = options or {}
        if bool(options.get("takeoff", False)):
            action = np.zeros(self.action_size)
            action[-1] = 1.0
            # Repeat across DDS discovery and preflight readiness; the flight
            # controller de-duplicates queued takeoff requests.
            for _ in range(3):
                self._publish_action(action)
                time.sleep(0.15)
        return observation, {
            "px4_in_the_loop": True,
            "gazebo_in_the_loop": True,
            "status": None if self._status is None else self._status.copy(),
        }

    def step(self, action: np.ndarray):
        sequence = self._sequence
        self._publish_action(action)
        observation = self._wait_observation(sequence)
        displacement = observation[:3].astype(float) - self._origin
        velocity = observation[3:6].astype(float)
        attitude = observation[6:9].astype(float)
        joint_velocity = observation[18:24].astype(float)
        reward = -(
            float(np.dot(displacement, displacement))
            + 0.1 * float(np.dot(velocity, velocity))
            + 0.2 * float(np.dot(attitude[:2], attitude[:2]))
            + 0.01 * float(np.dot(joint_velocity, joint_velocity))
        )
        status = None if self._status is None else self._status.copy()
        failsafe = bool(status is not None and status[2] > 0.5)
        terminated = bool(failsafe or np.linalg.norm(displacement) > 5.0)
        return observation, reward, terminated, False, {
            "px4_in_the_loop": True,
            "gazebo_in_the_loop": True,
            "status": status,
            "failsafe": failsafe,
        }

    def land(self) -> None:
        action = np.zeros(self.action_size)
        action[-1] = -1.0
        self._publish_action(action)

    def close(self) -> None:
        if self._closed:
            return
        self._closed = True
        self.executor.shutdown(timeout_sec=1.0)
        self.node.destroy_node()
        self.context.shutdown()
        self._thread.join(timeout=1.0)
