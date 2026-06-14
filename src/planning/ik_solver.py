from __future__ import annotations

import math

from src.common.config import DEFAULT_CONFIG, Config

_DH_A = (0.0, -0.42500, -0.39225, 0.0, 0.0, 0.0)
_DH_D = (0.089159, 0.0, 0.0, 0.10915, 0.09465, 0.08230)

# 缓存的 PyBullet IK 上下文，避免每次 solve_ik 都查 RUNTIME
_PYB_IK_CTX: dict = {}


def _get_pyb_ik_context() -> dict | None:
    """获取 PyBullet IK 所需资源，失败返回 None（回退到解析 IK）。"""
    global _PYB_IK_CTX
    if _PYB_IK_CTX:
        return _PYB_IK_CTX
    try:
        from src.simulation._runtime import RUNTIME, p
    except ImportError:
        return None
    if p is None:
        return None
    robot_id = RUNTIME.robot_id
    client_id = RUNTIME.client_id
    if robot_id is None or client_id is None:
        return None
    if not p.isConnected(client_id):
        return None
    # 找到 tool0 (优先) 或 ee_link 的 link 索引
    ee_idx = None
    joint_indices = []
    num = p.getNumJoints(robot_id, physicsClientId=client_id)
    for j in range(num):
        info = p.getJointInfo(robot_id, j, physicsClientId=client_id)
        link_name = info[12].decode("utf-8")
        joint_type = info[2]
        if joint_type in {p.JOINT_REVOLUTE, p.JOINT_PRISMATIC}:
            joint_indices.append(j)
        if link_name == "tool0" and ee_idx is None:
            ee_idx = j
    if ee_idx is None:
        # fallback: 最后一个 revolute 关节的 link (wrist_3_link)
        ee_idx = num - 5  # 跳过固定关节
    _PYB_IK_CTX = {
        "p": p,
        "robot_id": robot_id,
        "client_id": client_id,
        "ee_idx": ee_idx,
        "joint_indices": tuple(joint_indices),
    }
    return _PYB_IK_CTX


def solve_ik(target_xyz: tuple[float, float, float], config: Config = DEFAULT_CONFIG) -> tuple[float, ...]:
    """Solve UR5 inverse kinematics for a target point.

    当 PyBullet 可用时优先使用 PyBullet 内置 IK（保证与 URDF 模型运动学一致），
    失败或不可用时回退到解析 IK。
    """
    pyb_solution = _solve_ik_pybullet(target_xyz, config)
    if pyb_solution is not None:
        return pyb_solution

    target_pose = _target_pose_from_xyz(target_xyz, config)
    solutions = _inverse_kinematics_ur5(target_pose)
    if not solutions:
        return config.home_pose
    best = min(solutions, key=lambda solution: _distance_to_home(solution, config.base_link_position))
    return tuple(round(_wrap_to_pi(theta), 4) for theta in best)


def _solve_ik_pybullet(
    target_xyz: tuple[float, float, float],
    config: Config = DEFAULT_CONFIG,
) -> tuple[float, ...] | None:
    """使用 PyBullet 内置 IK 求解，保证与 URDF 模型运动学一致。

    接收世界坐标 target_xyz，内部转换为 robot base 系传给 PyBullet IK。
    EE link 使用「tool0」(joint 8)，转动关节为 1-6。
    失败时返回 None，调用方回退到解析 IK。
    """
    ctx = _get_pyb_ik_context()
    if ctx is None:
        return None

    p = ctx["p"]
    robot_id = ctx["robot_id"]
    client_id = ctx["client_id"]
    # 优先 tool0 (joint 8)，ee_link (joint 7) 备选
    ee_idx = ctx["ee_idx"]
    joint_indices = ctx["joint_indices"]

    # 目标在 robot base 系下的位置
    target_pos = [
        target_xyz[0] - config.base_link_position[0],
        target_xyz[1] - config.base_link_position[1],
        target_xyz[2] - config.base_link_position[2],
    ]

    # 工具姿态：x 同世界 x，y 反向世界 y，z 向下（= 绕 x 轴转 π）
    target_orn = p.getQuaternionFromEuler((math.pi, 0.0, 0.0))

    try:
        joint_angles = p.calculateInverseKinematics(
            robot_id,
            ee_idx,
            target_pos,
            target_orn,
            lowerLimits=[-math.pi] * 6,
            upperLimits=[math.pi] * 6,
            jointRanges=[2.0 * math.pi] * 6,
            restPoses=list(config.home_pose[:6]),
            physicsClientId=client_id,
        )
    except Exception:
        return None

    if len(joint_angles) < 6:
        return None

    return tuple(round(_wrap_to_pi(joint_angles[i]), 4) for i in range(6))


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
