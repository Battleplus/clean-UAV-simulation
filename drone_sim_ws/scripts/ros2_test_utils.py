"""Shared environment helpers for ROS 2 PTY regression runners."""

from __future__ import annotations

import os
from pathlib import Path


def ros2_child_environment() -> dict[str, str]:
    """Preserve ROS Python packages even when caller sets ``PYTHONPATH``.

    A command such as ``PYTHONPATH=src/drone_arm_sim python3 ...`` replaces,
    rather than extends, the shell's ROS ``PYTHONPATH``.  The child ``ros2``
    executable then cannot discover ``ros2cli``.  Rebuild the Python path from
    AMENT prefixes for the PTY child while retaining the caller's entries.
    """
    env = os.environ.copy()
    entries: list[str] = []
    for prefix in env.get("AMENT_PREFIX_PATH", "").split(os.pathsep):
        if not prefix:
            continue
        candidate = Path(prefix) / "lib" / "python3.12" / "site-packages"
        if candidate.is_dir():
            entries.append(str(candidate))
        # colcon's --symlink-install uses an egg-link in install/.  Python's
        # importlib.metadata may otherwise select an older real distribution
        # from a lower overlay even though ROS package lookup selects this
        # prefix.  Add the matching build tree explicitly so ros2 run loads
        # the same source that was just built.
        prefix_path = Path(prefix)
        if prefix_path.parent.name == "install":
            build_tree = prefix_path.parent.parent / "build" / prefix_path.name
            if build_tree.is_dir():
                entries.append(str(build_tree))
    existing = env.get("PYTHONPATH", "")
    if existing:
        entries.append(existing)
    if entries:
        env["PYTHONPATH"] = os.pathsep.join(dict.fromkeys(entries))
    return env
