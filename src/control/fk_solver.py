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

    优先使用 PyBullet FK（保证与 URDF 模型一致），
    PyBullet 不可用时回退到解析 DH FK。

    Args:
        joint_angles: 6 joint angles (rad)
        config: configuration for base_link_position

    Returns:
        (x, y, z) world-frame position of the end-effector
    """
    pyb_result = _solve_fk_pybullet(joint_angles)
    if pyb_result is not None:
        return pyb_result

    # ── 解析 DH FK (fallback) ──
    bx, by, bz = config.base_link_position
    T = _translation_matrix(bx, by, bz)

    for i in range(6):
        theta = joint_angles[i]
        d = _DH_D[i]
        a = _DH_A[i]
        alpha = _DH_ALPHA[i]
        T_i = _dh_transform(theta, d, a, alpha)
        T = _multiply_4x4(T, T_i)

    return (T[0][3], T[1][3], T[2][3])


# 缓存的 PyBullet FK 上下文
_PYB_FK_CTX: dict = {}


def _get_pyb_fk_context() -> dict | None:
    """获取 PyBullet FK 所需资源，失败返回 None（回退到解析 FK）。"""
    global _PYB_FK_CTX
    if _PYB_FK_CTX:
        return _PYB_FK_CTX
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
    # 找到 tool0 的 link 索引
    ee_idx = None
    joint_list = []
    num = p.getNumJoints(robot_id, physicsClientId=client_id)
    for j in range(num):
        info = p.getJointInfo(robot_id, j, physicsClientId=client_id)
        link_name = info[12].decode("utf-8")
        joint_type = info[2]
        if joint_type in {p.JOINT_REVOLUTE, p.JOINT_PRISMATIC}:
            joint_list.append(j)
        if link_name == "tool0" and ee_idx is None:
            ee_idx = j
    if ee_idx is None:
        ee_idx = num - 5
    _PYB_FK_CTX = {
        "p": p,
        "robot_id": robot_id,
        "client_id": client_id,
        "ee_idx": ee_idx,
        "joint_indices": tuple(joint_list),
    }
    return _PYB_FK_CTX


def _solve_fk_pybullet(joint_angles: tuple[float, ...]) -> tuple[float, float, float] | None:
    """使用 PyBullet getLinkState 计算 EE 世界坐标（无副作用）。"""
    ctx = _get_pyb_fk_context()
    if ctx is None:
        return None

    p = ctx["p"]
    robot_id = ctx["robot_id"]
    client_id = ctx["client_id"]
    ee_idx = ctx["ee_idx"]
    joint_indices = ctx["joint_indices"]  # 6 个转动关节索引 (1-6)

    # 保存当前关节状态
    saved = [
        p.getJointState(robot_id, j, physicsClientId=client_id)[0]
        for j in joint_indices
    ]

    # 临时设置关节到目标角度并读取 EE
    for j_idx, j in enumerate(joint_indices):
        p.resetJointState(robot_id, j, joint_angles[j_idx], physicsClientId=client_id)

    ee_state = p.getLinkState(robot_id, ee_idx, physicsClientId=client_id)
    result = (ee_state[0][0], ee_state[0][1], ee_state[0][2])

    # 恢复原始关节状态
    for j_idx, j in enumerate(joint_indices):
        p.resetJointState(robot_id, j, saved[j_idx], physicsClientId=client_id)

    return result


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
