from __future__ import annotations

import math

from src.common.config import DEFAULT_CONFIG, Config


_DH_A = (0.0, -0.42500, -0.39225, 0.0, 0.0, 0.0)
_DH_D = (0.089159, 0.0, 0.0, 0.10915, 0.09465, 0.08230)


def solve_ik(target_xyz: tuple[float, float, float], config: Config = DEFAULT_CONFIG) -> tuple[float, ...]:
    """Solve UR5 inverse kinematics for a target point using an analytic backend.

    The translated MATLAB script solves a full 4x4 target pose. This project API
    currently provides only position, so we use a fixed tool orientation with the
    tool z-axis pointing downward toward the board.
    """
    target_pose = _target_pose_from_xyz(target_xyz, config)
    solutions = _inverse_kinematics_ur5(target_pose)
    if not solutions:
        return config.home_pose
    best = min(solutions, key=lambda solution: _distance_to_home(solution, config.base_link_position))
    return tuple(round(_wrap_to_pi(theta), 4) for theta in best)


def is_reachable(target_xyz: tuple[float, float, float], config: Config = DEFAULT_CONFIG) -> bool:
    """Basic workspace guard for early CLI/GUI feedback."""
    base_x, base_y = config.base_link_position[:2]
    target_x, target_y = target_xyz[:2]
    distance = math.hypot(target_x - base_x, target_y - base_y)
    return bool(0.25 <= distance <= 0.9)


def _target_pose_from_xyz(target_xyz: tuple[float, float, float], config: Config) -> list[list[float]]:
    """Build a UR5-base-frame pose from a world-frame point.

    Orientation convention: x points along world x, y points opposite world y,
    and z points downward. This is a simple fixed grasp posture for board pieces.
    """
    px = target_xyz[0] - config.base_link_position[0]
    py = target_xyz[1] - config.base_link_position[1]
    pz = target_xyz[2] - config.base_link_position[2]
    return [
        [1.0, 0.0, 0.0, px],
        [0.0, -1.0, 0.0, py],
        [0.0, 0.0, -1.0, pz],
        [0.0, 0.0, 0.0, 1.0],#和世界坐标系相比，x轴同向，y反向，z反向
    ]


def _inverse_kinematics_ur5(target_pose: list[list[float]]) -> list[tuple[float, ...]]:
    """Translate the MATLAB UR5 standard-DH analytic IK into Python.

    Returns up to eight candidate joint solutions. Invalid branches caused by
    unreachable geometry or singular divisions are skipped.
    """
    t = target_pose
    nx, ny, nz = t[0][0], t[1][0], t[2][0]
    ox, oy, oz = t[0][1], t[1][1], t[2][1]
    ax, ay, az = t[0][2], t[1][2], t[2][2]
    px, py, pz = t[0][3], t[1][3], t[2][3]

    a2 = _DH_A[1]
    a3 = _DH_A[2]
    d1 = _DH_D[0]
    d4 = _DH_D[3]
    d5 = _DH_D[4]
    d6 = _DH_D[5]

    m = d6 * ay - py
    n = ax * d6 - px
    shoulder_radical = m * m + n * n - d4 * d4
    if shoulder_radical < -1e-9:
        return []
    shoulder_root = math.sqrt(max(0.0, shoulder_radical))
    theta1_candidates = [
        math.atan2(m, n) - math.atan2(d4, shoulder_root),
        math.atan2(m, n) - math.atan2(d4, -shoulder_root),
    ]

    solutions: list[tuple[float, ...]] = []
    for theta1 in theta1_candidates:
        c1 = math.cos(theta1)
        s1 = math.sin(theta1)
        wrist_cos = _clamp(ax * s1 - ay * c1)
        theta5_candidates = [math.acos(wrist_cos), -math.acos(wrist_cos)]
        mm = nx * s1 - ny * c1
        nn = ox * s1 - oy * c1

        for theta5 in theta5_candidates:
            theta6 = math.atan2(mm, nn) - math.atan2(math.sin(theta5), 0.0)
            c6 = math.cos(theta6)
            s6 = math.sin(theta6)
            radial = (
                d5 * (s6 * (nx * c1 + ny * s1) + c6 * (ox * c1 + oy * s1))
                - d6 * (ax * c1 + ay * s1)
                + px * c1
                + py * s1
            )
            vertical = pz - d1 - az * d6 + d5 * (oz * c6 + nz * s6)
            cos3_raw = (radial * radial + vertical * vertical - a2 * a2 - a3 * a3) / (2.0 * a2 * a3)
            if cos3_raw < -1.0 - 1e-9 or cos3_raw > 1.0 + 1e-9:
                continue
            cos3 = _clamp(cos3_raw)

            for theta3 in (math.acos(cos3), -math.acos(cos3)):
                denominator = a2 * a2 + a3 * a3 + 2.0 * a2 * a3 * math.cos(theta3)
                if abs(denominator) < 1e-9:
                    continue
                sin3 = math.sin(theta3)
                cos3_theta = math.cos(theta3)
                s2 = ((a3 * cos3_theta + a2) * vertical - a3 * sin3 * radial) / denominator
                c2_denominator = a3 * cos3_theta + a2
                if abs(c2_denominator) < 1e-9:
                    continue
                c2 = (radial + a3 * sin3 * s2) / c2_denominator
                theta2 = math.atan2(s2, c2)
                theta4 = (
                    math.atan2(
                        -math.sin(theta6) * (nx * c1 + ny * s1) - math.cos(theta6) * (ox * c1 + oy * s1),
                        oz * math.cos(theta6) + nz * math.sin(theta6),
                    )
                    - theta2
                    - theta3
                )
                solution = (theta1, theta2, theta3, theta4, theta5, theta6)
                if all(math.isfinite(theta) for theta in solution):
                    solutions.append(solution)
    return solutions


def _distance_to_home(solution: tuple[float, ...], home_pose: tuple[float, ...]) -> float:
    return math.sqrt(sum((theta - home) ** 2 for theta, home in zip(solution, home_pose[:6])))


def _clamp(value: float, lower: float = -1.0, upper: float = 1.0) -> float:
    return min(upper, max(lower, value))


def _wrap_to_pi(angle: float) -> float:
    return (angle + math.pi) % (2.0 * math.pi) - math.pi
