from __future__ import annotations

from src.common.config import DEFAULT_CONFIG, Config
from src.common.types import JointTrajectory, MotionPrimitive, Obstacle
from src.planning.ik_solver import solve_ik


def plan_trajectory(
    primitives: list[MotionPrimitive],
    obstacles: list[Obstacle],
    config: Config = DEFAULT_CONFIG,
) -> JointTrajectory:
    """Plan a mock joint trajectory and Fast/Safe speed profile."""
    joint_waypoints: list[tuple[float, ...]] = []
    speed_profile: list[str] = []
    for primitive in primitives:
        joint_waypoints.append(solve_ik(primitive.target_xyz, config))
        speed_profile.append(_choose_speed_mode(primitive, obstacles))
    return JointTrajectory(joint_waypoints=joint_waypoints, speed_profile=speed_profile)


def _choose_speed_mode(primitive: MotionPrimitive, obstacles: list[Obstacle]) -> str:
    if primitive.speed_mode == "safe":
        return "safe"
    x, y, _ = primitive.target_xyz
    for obstacle in obstacles:
        ox, oy, _ = obstacle.center_xyz
        distance_xy = ((x - ox) ** 2 + (y - oy) ** 2) ** 0.5
        if distance_xy < obstacle.radius + 0.03:
            return "safe"
    return "fast"
