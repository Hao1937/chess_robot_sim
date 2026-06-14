from __future__ import annotations

import math

from src.common.config import DEFAULT_CONFIG, Config


def solve_fk(
    joint_angles: tuple[float, ...],
    config: Config = DEFAULT_CONFIG,
) -> tuple[float, float, float]:
    """UR5 forward kinematics: joint angles → end-effector world position.

    优先使用 PyBullet FK（保证与 URDF 模型一致），
    PyBullet 不可用时回退到 URDF 链 FK。

    Args:
        joint_angles: 6 joint angles (rad)
        config: configuration for base_link_position

    Returns:
        (x, y, z) world-frame position of the end-effector (tool0)
    """
    pyb_result = _solve_fk_pybullet(joint_angles)
    if pyb_result is not None:
        return pyb_result

    # ── URDF chain FK (fallback) ──
    T = _solve_fk_urdf_chain_matrix(joint_angles, config)
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


# ── URDF chain FK (matches ur5_joint_limited_robot.urdf exactly) ──


def _solve_fk_urdf_chain_matrix(
    joint_angles: tuple[float, ...],
    config: Config = DEFAULT_CONFIG,
) -> list[list[float]]:
    """Compute the full 4×4 homogeneous transform of tool0 in world frame.

    The chain follows `ur5_joint_limited_robot.urdf`:
      base_link → shoulder_pan(z) → shoulder_lift(y) → elbow(y)
      → wrist_1(y) → wrist_2(z) → wrist_3(y) → tool0(fixed)
    """
    θ0, θ1, θ2, θ3, θ4, θ5 = joint_angles

    bx, by, bz = config.base_link_position
    T = _translation_matrix(bx, by, bz)

    # Joint 0: shoulder_pan (z-axis)
    #   origin: xyz=(0, 0, 0.089159), rpy=(0, 0, 0)
    T = _multiply_4x4(T, _joint_z(θ0, 0.0, 0.0, 0.089159))

    # Joint 1: shoulder_lift (y-axis)
    #   origin: xyz=(0, 0.13585, 0), rpy=(0, π/2, 0)
    T = _multiply_4x4(T, _joint_y(θ1, 0.0, 0.13585, 0.0, math.pi / 2))

    # Joint 2: elbow (y-axis)
    #   origin: xyz=(0, -0.1197, 0.425), rpy=(0, 0, 0)
    T = _multiply_4x4(T, _joint_y(θ2, 0.0, -0.1197, 0.425, 0.0))

    # Joint 3: wrist_1 (y-axis)
    #   origin: xyz=(0, 0, 0.39225), rpy=(0, π/2, 0)
    T = _multiply_4x4(T, _joint_y(θ3, 0.0, 0.0, 0.39225, math.pi / 2))

    # Joint 4: wrist_2 (z-axis)
    #   origin: xyz=(0, 0.093, 0), rpy=(0, 0, 0)
    T = _multiply_4x4(T, _joint_z(θ4, 0.0, 0.093, 0.0))

    # Joint 5: wrist_3 (y-axis)
    #   origin: xyz=(0, 0, 0.09465), rpy=(0, 0, 0)
    T = _multiply_4x4(T, _joint_y(θ5, 0.0, 0.0, 0.09465, 0.0))

    # Fixed joint: wrist_3 → tool0
    #   origin: xyz=(0, 0.0823, 0), rpy=(-π/2, 0, 0)
    T = _multiply_4x4(T, _fixed_joint(0.0, 0.0823, 0.0, -math.pi / 2, 0.0, 0.0))

    return T


def _solve_fk_urdf_chain(
    joint_angles: tuple[float, ...],
    config: Config = DEFAULT_CONFIG,
) -> tuple[float, float, float]:
    """Compute tool0 position by walking the URDF kinematic chain directly."""
    T = _solve_fk_urdf_chain_matrix(joint_angles, config)
    return (T[0][3], T[1][3], T[2][3])


def _get_tool0_z_axis(
    joint_angles: tuple[float, ...],
    config: Config = DEFAULT_CONFIG,
) -> tuple[float, float, float]:
    """Return tool0 z-axis direction in world frame (unit vector).

    The tool points DOWN when z_axis ≈ (0, 0, -1).
    """
    T = _solve_fk_urdf_chain_matrix(joint_angles, config)
    # Rotation matrix columns: x=T[0:3][0], y=T[0:3][1], z=T[0:3][2]
    return (T[0][2], T[1][2], T[2][2])


# ── internal helpers ──


def _translation_matrix(x: float, y: float, z: float) -> list[list[float]]:
    return [
        [1.0, 0.0, 0.0, x],
        [0.0, 1.0, 0.0, y],
        [0.0, 0.0, 1.0, z],
        [0.0, 0.0, 0.0, 1.0],
    ]


def _rotation_x(a: float) -> list[list[float]]:
    """Rotation about x-axis by a radians."""
    ca = math.cos(a)
    sa = math.sin(a)
    return [
        [1.0, 0.0, 0.0, 0.0],
        [0.0,  ca, -sa, 0.0],
        [0.0,  sa,  ca, 0.0],
        [0.0, 0.0, 0.0, 1.0],
    ]


def _rotation_y(a: float) -> list[list[float]]:
    """Rotation about y-axis by a radians."""
    ca = math.cos(a)
    sa = math.sin(a)
    return [
        [ ca, 0.0,  sa, 0.0],
        [0.0, 1.0, 0.0, 0.0],
        [-sa, 0.0,  ca, 0.0],
        [0.0, 0.0, 0.0, 1.0],
    ]


def _rotation_z(a: float) -> list[list[float]]:
    """Rotation about z-axis by a radians."""
    ca = math.cos(a)
    sa = math.sin(a)
    return [
        [ ca, -sa, 0.0, 0.0],
        [ sa,  ca, 0.0, 0.0],
        [0.0, 0.0, 1.0, 0.0],
        [0.0, 0.0, 0.0, 1.0],
    ]


def _rpy_matrix(rx: float, ry: float, rz: float) -> list[list[float]]:
    """Fixed-axis RPY rotation: Rz(rz)·Ry(ry)·Rx(rx)."""
    R = _rotation_z(rz)
    R = _multiply_4x4(R, _rotation_y(ry))
    R = _multiply_4x4(R, _rotation_x(rx))
    return R


def _joint_z(
    theta: float, dx: float, dy: float, dz: float
) -> list[list[float]]:
    """Transform for a z-axis revolute joint: T(dx,dy,dz)·Rz(theta)."""
    T = _translation_matrix(dx, dy, dz)
    return _multiply_4x4(T, _rotation_z(theta))


def _joint_y(
    theta: float, dx: float, dy: float, dz: float, rpy_y: float
) -> list[list[float]]:
    """Transform for a y-axis revolute joint with optional RPY offset.

    URDF convention: T(dx,dy,dz)·Ry(rpy_y)·Ry(theta)
    The rpy_y is the fixed RPY component (e.g. π/2 for shoulder_lift origin).
    """
    T = _translation_matrix(dx, dy, dz)
    T = _multiply_4x4(T, _rotation_y(rpy_y))
    return _multiply_4x4(T, _rotation_y(theta))


def _fixed_joint(
    dx: float, dy: float, dz: float, rx: float, ry: float, rz: float
) -> list[list[float]]:
    """Transform for a fixed joint: T(dx,dy,dz)·Rz(rz)·Ry(ry)·Rx(rx)."""
    T = _translation_matrix(dx, dy, dz)
    return _multiply_4x4(T, _rpy_matrix(rx, ry, rz))


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
