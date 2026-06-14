from __future__ import annotations

import math

from src.common.config import DEFAULT_CONFIG, Config

# UR5 standard DH parameters (same as ik_solver.py)
_DH_A = (0.0, -0.42500, -0.39225, 0.0, 0.0, 0.0)
_DH_D = (0.089159, 0.0, 0.0, 0.10915, 0.09465, 0.08230)
_DH_ALPHA = (math.pi / 2, 0.0, 0.0, math.pi / 2, -math.pi / 2, 0.0)


def solve_fk(
    joint_angles: tuple[float, ...],
    config: Config = DEFAULT_CONFIG,
) -> tuple[float, float, float]:
    """UR5 forward kinematics: joint angles → end-effector world position.

    Uses the standard DH convention (same parameters as ik_solver.py).
    The FK chain goes: base → joint1 → … → joint6 → tool.

    The tool frame (joint6 output) uses the same convention as
    ``_target_pose_from_xyz``: x along world x, y opposite world y,
    z downward.  We extract the tool's origin and apply the base_link
    offset to return a world-frame position.

    Args:
        joint_angles: 6 joint angles (rad)
        config: configuration for base_link_position

    Returns:
        (x, y, z) world-frame position of the end-effector
    """
    # Start from world frame → base_link frame
    bx, by, bz = config.base_link_position

    # Identity matrix in world frame; then translate to base_link
    T = _translation_matrix(bx, by, bz)

    for i in range(6):
        theta = joint_angles[i]
        d = _DH_D[i]
        a = _DH_A[i]
        alpha = _DH_ALPHA[i]

        # Standard DH: T_i = Rz(theta) * Tz(d) * Tx(a) * Rx(alpha)
        T_i = _dh_transform(theta, d, a, alpha)
        T = _multiply_4x4(T, T_i)

    # T is now the tool frame in world coordinates
    # The tool origin is the translation column of T
    return (T[0][3], T[1][3], T[2][3])


def compute_ee_error(
    desired_joint: tuple[float, ...],
    actual_joint: tuple[float, ...],
    config: Config = DEFAULT_CONFIG,
) -> float:
    """Compute Euclidean end-effector position error between desired and actual joints.

    Args:
        desired_joint: desired joint angles (6 floats)
        actual_joint: actual joint angles (6 floats)
        config: configuration

    Returns:
        Euclidean distance (m) between desired and actual EE positions
    """
    desired_xyz = solve_fk(desired_joint, config)
    actual_xyz = solve_fk(actual_joint, config)
    return math.hypot(
        desired_xyz[0] - actual_xyz[0],
        desired_xyz[1] - actual_xyz[1],
        desired_xyz[2] - actual_xyz[2],
    )


# ── internal helpers ──


def _dh_transform(
    theta: float, d: float, a: float, alpha: float
) -> list[list[float]]:
    """Standard DH transformation matrix: Rz(θ)·Tz(d)·Tx(a)·Rx(α)."""
    ct = math.cos(theta)
    st = math.sin(theta)
    ca = math.cos(alpha)
    sa = math.sin(alpha)

    return [
        [ct, -st * ca,  st * sa, a * ct],
        [st,  ct * ca, -ct * sa, a * st],
        [0.0,      sa,      ca,      d],
        [0.0,     0.0,     0.0,    1.0],
    ]


def _translation_matrix(x: float, y: float, z: float) -> list[list[float]]:
    return [
        [1.0, 0.0, 0.0, x],
        [0.0, 1.0, 0.0, y],
        [0.0, 0.0, 1.0, z],
        [0.0, 0.0, 0.0, 1.0],
    ]


def _multiply_4x4(
    A: list[list[float]], B: list[list[float]]
) -> list[list[float]]:
    """4×4 matrix multiplication."""
    result = [[0.0] * 4 for _ in range(4)]
    for i in range(4):
        for k in range(4):
            aik = A[i][k]
            for j in range(4):
                result[i][j] += aik * B[k][j]
    return result
