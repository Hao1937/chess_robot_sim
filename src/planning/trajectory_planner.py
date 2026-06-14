from __future__ import annotations

from src.common.config import DEFAULT_CONFIG, Config
from src.common.types import JointTrajectory, MotionPrimitive, Obstacle, PrimitivePlanningContext
from src.planning.ik_solver import solve_ik


def plan_trajectory(
    primitives: list[MotionPrimitive] | list[PrimitivePlanningContext],
    obstacles: list[Obstacle] | None = None,
    config: Config = DEFAULT_CONFIG,
) -> JointTrajectory:
    """Plan a mock joint trajectory and Fast/Safe speed profile."""
    joint_waypoints: list[tuple[float, ...]] = []
    speed_profile: list[str] = []
    for item in primitives:
        if isinstance(item, PrimitivePlanningContext):
            primitive = item.primitive
            primitive_obstacles = item.obstacles
        else:
            primitive = item
            primitive_obstacles = obstacles or []
        joint_waypoints.append(solve_ik(primitive.target_xyz, config))
        speed_profile.append(_choose_speed_mode(primitive, primitive_obstacles))
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
