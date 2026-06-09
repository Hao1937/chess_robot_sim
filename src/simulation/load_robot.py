from __future__ import annotations

from src.common.types import RobotHandle


def load_robot(urdf_path: str | None = None) -> RobotHandle:
    """Load a robot model.

    The skeleton returns a mock handle. B can replace the body with PyBullet
    loading while keeping the same return type.
    """
    return RobotHandle(robot_id=1, end_effector_id=6, joint_indices=(0, 1, 2, 3, 4, 5))
