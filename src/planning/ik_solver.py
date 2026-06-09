from __future__ import annotations

from src.common.config import DEFAULT_CONFIG, Config


def solve_ik(target_xyz: tuple[float, float, float], config: Config = DEFAULT_CONFIG) -> tuple[float, ...]:
    """Return a mock joint solution for a target point.

    C can replace this with PyBullet IK while preserving the signature.
    """
    x, y, z = target_xyz
    return (round(x, 4), round(y, 4), round(z, 4), -0.5, 0.0, 0.0)


def is_reachable(target_xyz: tuple[float, float, float], config: Config = DEFAULT_CONFIG) -> bool:
    """Basic workspace guard for early CLI/GUI feedback."""
    x, y, z = target_xyz
    return -0.1 <= x <= 0.6 and -0.1 <= y <= 0.6 and 0.0 <= z <= 0.4
