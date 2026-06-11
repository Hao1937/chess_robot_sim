from __future__ import annotations

import math

from src.common.config import DEFAULT_CONFIG, Config
from src.simulation.load_robot import load_robot
from src.simulation._runtime import RUNTIME, ensure_client, p


def solve_ik(target_xyz: tuple[float, float, float], config: Config = DEFAULT_CONFIG) -> tuple[float, ...]:
    """Return a mock joint solution for a target point.
    C can replace this with PyBullet IK while preserving the signature.
    """
    if p is None:
        return config.home_pose

    client_id = ensure_client()
    robot = load_robot()
    joint_solution = p.calculateInverseKinematics(
        robot.robot_id,
        robot.end_effector_id,
        target_xyz,
        physicsClientId=client_id if client_id is not None else RUNTIME.client_id,
    )
    return tuple(round(theta, 4) for theta in joint_solution[:6])


def is_reachable(target_xyz: tuple[float, float, float], config: Config = DEFAULT_CONFIG) -> bool:
    """Basic workspace guard for early CLI/GUI feedback."""
#完全伸直0.9 完全蜷缩0.25
    base_x, base_y = config.base_link_position[:2]
    target_x, target_y = target_xyz[:2]
    distance = math.hypot(target_x - base_x, target_y - base_y)
    return bool(0.25<=distance<=0.9)
