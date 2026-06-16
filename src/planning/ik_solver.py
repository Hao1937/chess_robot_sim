from __future__ import annotations

import math

from src.common.config import DEFAULT_CONFIG, Config


def current_joint_seed(config: Config = DEFAULT_CONFIG) -> tuple[float, ...]:
    """返回机器人当前实际关节角，作为 IK 链的起始种子。

    PyBullet 已连接且机器人已加载时读取实时关节状态；否则回退到 home_pose。

    设计依据：IK 链必须从机器人**当前物理姿态**播种，而非固定的 home_pose。
    否则每条命令都假设从 home 出发，规划轨迹起点会脱离机器人实际位置，
    导致段间巨大跳变 / 落入远分支（详见 controller 调试经验）。
    """
    fallback = tuple(config.home_pose[:6])
    try:
        from src.simulation._runtime import RUNTIME, p
    except ImportError:
        return fallback
    if p is None:
        return fallback
    robot_id = RUNTIME.robot_id
    client_id = RUNTIME.client_id
    if robot_id is None or client_id is None or not RUNTIME.joint_indices:
        return fallback
    if not p.isConnected(client_id):
        return fallback
    try:
        return tuple(
            p.getJointState(robot_id, j, physicsClientId=client_id)[0]
            for j in RUNTIME.joint_indices[:6]
        )
    except Exception:
        return fallback


def solve_ik(
    target_xyz: tuple[float, float, float],
    config: Config = DEFAULT_CONFIG,
    *,
    seed: tuple[float, ...] | None = None,
) -> tuple[float, ...]:
    """Solve UR5 inverse kinematics for a target point.

    统一使用确定性数值 IK（damped least squares + 零空间朝向优化）。

    为什么不用 PyBullet 内置 IK：`calculateInverseKinematics` 以机器人**实时
    关节状态**作迭代种子（非确定），且只能验证位置无法保证朝向 → 同一目标在
    不同物理姿态下给出不同/吸盘倾斜的解，是「第二步走棋位姿失控」的根因。
    数值 IK 是 (target, seed) 的纯函数、链式连续、强制 -zz>0.92 竖直，且其 FK
    用 `_solve_fk_urdf_chain` 与 URDF 精确一致——已覆盖 PyBullet IK 的唯一卖点。

    Args:
        target_xyz: 目标 EE 世界坐标
        config: 配置对象
        seed: 可选的初始关节猜测（6 元组），用于确保邻近路径点的解分支连续性
    """
    return _solve_ik_numerical(target_xyz, config, seed=seed)


def is_reachable(target_xyz: tuple[float, float, float], config: Config = DEFAULT_CONFIG) -> bool:
    """Basic workspace guard for early CLI/GUI feedback."""
    base_x, base_y = config.base_link_position[:2]
    target_x, target_y = target_xyz[:2]
    distance = math.hypot(target_x - base_x, target_y - base_y)
    return bool(0.25 <= distance <= 0.9)


# ── Numerical IK (Jacobian pseudo-inverse with damped least squares) ──

# Joint limits from the URDF
_JOINT_LIMITS = [
    (-2.0 * math.pi, 2.0 * math.pi),
    (-2.0 * math.pi, 2.0 * math.pi),
    (-2.0 * math.pi, 2.0 * math.pi),
    (-2.0 * math.pi, 2.0 * math.pi),
    (-2.0 * math.pi, 2.0 * math.pi),
    (-2.0 * math.pi, 2.0 * math.pi),
]


# A candidate must be accurate and nearly vertical-down before it can win early.
_IK_GOOD_POSITION_TOL = 0.006
_IK_DOWNWARD_TARGET = 0.92
# 备选种子：覆盖不同 shoulder_pan/lift/elbow 分支，帮助跳出局部极小
_FALLBACK_IK_SEEDS: tuple[tuple[float, ...], ...] = (
    (0.0, -1.2, 1.8, -0.6, 0.0, 0.0),
    (0.0, -2.0, 2.0, -1.5, 0.0, 0.0),
    (0.5, -1.0, 1.5, -0.8, 0.0, 0.0),
    (-0.5, -1.0, 1.5, -0.8, 0.0, 0.0),
)


def _solve_ik_numerical(
    target_xyz: tuple[float, float, float],
    config: Config = DEFAULT_CONFIG,
    *,
    max_iters: int = 300,
    tolerance: float = 0.001,
    damping: float = 0.15,
    seed: tuple[float, ...] | None = None,
) -> tuple[float, ...]:
    """Numerical IK using damped least squares Jacobian pseudo-inverse.

    使用 URDF 链 FK 保证与 PyBullet 模型完全一致。

    多种子鲁棒性：先从 seed（或 home）下降；若收敛失败（best_error 偏大），
    依次换备选种子重解，取 best_error 最小者。这消除了「链式种子陷入局部极小
    → 返回偏差数 cm 的解 → 可行性闸门误判不可达」的问题。常见路径（主种子即收敛）
    只跑一次下降，无额外开销。

    Args:
        target_xyz: target EE position in world frame
        config: configuration
        max_iters: maximum iterations
        tolerance: convergence threshold (m)
        damping: damping factor for singularity robustness
        seed: optional initial joint guess for consistent solution branches

    Returns:
        6 joint angles (rad), or seed/home_pose if no solution found
    """
    # 种子顺序：优先用传入 seed（保链式连续），再 home，再备选
    seeds: list[tuple[float, ...]] = []
    if seed is not None:
        seeds.append(tuple(seed))
    seeds.append(tuple(config.home_pose[:6]))
    seeds.extend(_FALLBACK_IK_SEEDS)

    from src.control.fk_solver import _get_tool0_z_axis, _solve_fk_urdf_chain

    primary_seed = tuple(seed) if seed is not None else tuple(config.home_pose[:6])
    best_solution: tuple[float, ...] | None = None
    best_score = float("inf")
    best_error = float("inf")
    for seed_i in seeds:
        theta_i, err_i = _ik_descent(
            target_xyz, config, seed_i, max_iters, tolerance, damping,
        )
        if err_i >= 0.05:
            if err_i < best_error:
                best_error = err_i
                best_solution = tuple(theta_i)
            continue

        candidate = _nullspace_optimize_orientation(tuple(theta_i), target_xyz, config)
        fk_xyz = _solve_fk_urdf_chain(candidate, config)
        final_error = math.sqrt(sum((fk_xyz[i] - target_xyz[i]) ** 2 for i in range(3)))
        _, _, zz = _get_tool0_z_axis(candidate, config)
        downward_score = -zz

        if final_error < best_error:
            best_error = final_error

        continuity_cost = _joint_distance_wrapped(candidate, primary_seed)
        orientation_shortfall = max(0.0, _IK_DOWNWARD_TARGET - downward_score)
        score = final_error + 0.08 * orientation_shortfall * orientation_shortfall + 0.0005 * continuity_cost
        if score < best_score:
            best_score = score
            best_solution = candidate

        if final_error <= _IK_GOOD_POSITION_TOL and downward_score >= _IK_DOWNWARD_TARGET:
            return candidate

    # 收敛成功 → 返回姿态/位置/连续性综合评分最好的解
    if best_solution is not None and best_error < 0.05:
        return best_solution

    # 所有种子均未收敛：返回 seed（保轨迹连续）或 home_pose
    if seed is not None:
        return tuple(seed)
    return config.home_pose[:6]


def _ik_descent(
    target_xyz: tuple[float, float, float],
    config: Config,
    theta0: tuple[float, ...],
    max_iters: int,
    tolerance: float,
    damping: float,
) -> tuple[list[float], float]:
    """单次 DLS 下降（带零空间朝向优化方向），返回 (best_theta, best_error)。

    不在此做最终朝向精修——由调用方对全局最优解统一做
    ``_nullspace_optimize_orientation``，避免对每个种子重复精修。
    """
    from src.control.fk_solver import _solve_fk_urdf_chain

    theta = list(theta0)
    best_theta = list(theta)
    best_error = float("inf")

    for _ in range(max_iters):
        current = _solve_fk_urdf_chain(tuple(theta), config)
        error_vec = [
            target_xyz[0] - current[0],
            target_xyz[1] - current[1],
            target_xyz[2] - current[2],
        ]
        error = math.sqrt(sum(e**2 for e in error_vec))

        if error < best_error:
            best_error = error
            best_theta = list(theta)

        if error < tolerance:
            break

        J = _compute_jacobian(tuple(theta), config)
        delta_pos = _dls_step(J, error_vec, damping)
        g_orient = _orientation_gradient(tuple(theta), config)
        delta_null = _nullspace_project(J, delta_pos, g_orient, damping)

        for j in range(6):
            theta[j] += delta_null[j]
            lo, hi = _JOINT_LIMITS[j]
            theta[j] = max(lo, min(hi, theta[j]))

        # 自适应阻尼
        if error > 0.1:
            damping = 0.3
        elif error > 0.01:
            damping = 0.15
        else:
            damping = 0.05

    return best_theta, best_error


def _compute_jacobian(
    theta: tuple[float, ...],
    config: Config,
    delta: float = 0.0005,
) -> list[list[float]]:
    """Compute 3×6 position Jacobian numerically using central differences.

    J[i][j] = ∂x_i / ∂θ_j
    """
    from src.control.fk_solver import _solve_fk_urdf_chain

    J = [[0.0] * 6 for _ in range(3)]  # 3 rows × 6 cols

    f0 = _solve_fk_urdf_chain(theta, config)

    for j in range(6):
        theta_plus = list(theta)
        theta_plus[j] += delta
        f_plus = _solve_fk_urdf_chain(tuple(theta_plus), config)

        # Forward difference (accurate enough with small delta)
        for i in range(3):
            J[i][j] = (f_plus[i] - f0[i]) / delta

    return J


def _dls_step(
    J: list[list[float]],  # 3×6
    error_vec: list[float],  # 3
    damping: float,
) -> list[float]:  # 6
    """Damped least squares: Δθ = J^T (J·J^T + λ²·I)⁻¹ · e

    J is 3×6, so J·J^T is 3×3.
    """
    # J·J^T (3×3)
    JJT = [[0.0] * 3 for _ in range(3)]
    for i in range(3):
        for k in range(3):
            s = 0.0
            for j in range(6):
                s += J[i][j] * J[k][j]
            JJT[i][k] = s

    # J·J^T + λ²·I
    lam_sq = damping * damping
    A = [[JJT[i][k] + (lam_sq if i == k else 0.0) for k in range(3)] for i in range(3)]

    # Solve A · x = error_vec for x (3×1)
    x = _solve_3x3(A, error_vec)

    # Δθ = J^T · x (6×1)
    delta = [0.0] * 6
    for j in range(6):
        s = 0.0
        for i in range(3):
            s += J[i][j] * x[i]
        delta[j] = s

    return delta


def _solve_3x3(A: list[list[float]], b: list[float]) -> list[float]:
    """Solve 3×3 linear system A·x = b using Cramer's rule.

    Returns zero vector if singular.
    """
    # Determinant of 3×3
    def det3(M):
        return (
            M[0][0] * (M[1][1] * M[2][2] - M[1][2] * M[2][1])
            - M[0][1] * (M[1][0] * M[2][2] - M[1][2] * M[2][0])
            + M[0][2] * (M[1][0] * M[2][1] - M[1][1] * M[2][0])
        )

    d = det3(A)
    if abs(d) < 1e-15:
        return [0.0, 0.0, 0.0]

    # Replace each column with b
    A0 = [[b[0], A[0][1], A[0][2]], [b[1], A[1][1], A[1][2]], [b[2], A[2][1], A[2][2]]]
    A1 = [[A[0][0], b[0], A[0][2]], [A[1][0], b[1], A[1][2]], [A[2][0], b[2], A[2][2]]]
    A2 = [[A[0][0], A[0][1], b[0]], [A[1][0], A[1][1], b[1]], [A[2][0], A[2][1], b[2]]]

    return [det3(A0) / d, det3(A1) / d, det3(A2) / d]


# ── legacy helpers (保留兼容性) ──


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
        [0.0, 0.0, 0.0, 1.0],
    ]


def _inverse_kinematics_ur5(target_pose: list[list[float]]) -> list[tuple[float, ...]]:
    """Legacy DH-based analytical IK — 保留以供参考和向后兼容。

    注意：此函数基于 Standard DH 参数，与 URDF 模型运动学不完全一致。
    新代码应使用 solve_ik() 入口，它会优先使用 PyBullet IK，
    失败时回退到数值 IK（_solve_ik_numerical）。
    """
    # Standard DH parameters (kept for reference)
    _DH_A = (0.0, -0.42500, -0.39225, 0.0, 0.0, 0.0)
    _DH_D = (0.089159, 0.0, 0.0, 0.10915, 0.09465, 0.08230)

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


def _joint_distance_wrapped(solution: tuple[float, ...], seed: tuple[float, ...]) -> float:
    return math.sqrt(
        sum(_wrap_to_pi(theta - seed_theta) ** 2 for theta, seed_theta in zip(solution, seed[:6]))
    )


def _clamp(value: float, lower: float = -1.0, upper: float = 1.0) -> float:
    return min(upper, max(lower, value))


def _wrap_to_pi(angle: float) -> float:
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def _orientation_gradient(
    theta: tuple[float, ...],
    config: Config,
    delta: float = 0.002,
) -> list[float]:
    """Gradient of downward score = -zz w.r.t. each joint angle.

    Returns a 6-vector: g[j] = ∂(-zz)/∂θj
    """
    from src.control.fk_solver import _get_tool0_z_axis

    _, _, zz0 = _get_tool0_z_axis(theta, config)
    grad = [0.0] * 6
    for j in range(6):
        t_plus = list(theta)
        t_plus[j] += delta
        _, _, zz_p = _get_tool0_z_axis(tuple(t_plus), config)
        grad[j] = (-zz_p - (-zz0)) / delta
    return grad


def _nullspace_project(
    J: list[list[float]],       # 3×6 position Jacobian
    delta_pos: list[float],     # 6-vector from DLS
    g_orient: list[float],      # 6-vector orientation gradient
    damping: float,
) -> list[float]:
    """Project orientation gradient into position Jacobian null-space.

    Computes:
      J⁺ = J^T (J·J^T + λ²I)⁻¹    (damped pseudo-inverse, 6×3)
      N  = I - J⁺·J                (null-space projector, 6×6)
      Δθ = Δθ_pos + α · N · g_orient

    This uses the arm's redundant DOF to optimise orientation
    without disturbing the end-effector position.
    """
    # J·J^T (3×3)
    JJT = [[0.0] * 3 for _ in range(3)]
    for i in range(3):
        for k in range(3):
            s = 0.0
            for j in range(6):
                s += J[i][j] * J[k][j]
            JJT[i][k] = s

    lam_sq = damping * damping
    A = [[JJT[i][k] + (lam_sq if i == k else 0.0) for k in range(3)] for i in range(3)]

    # J⁺ = J^T · A⁻¹  (6×3)
    # For each column of A⁻¹, compute J^T times that column
    # We need J⁺ applied to (J · g_orient) efficiently:
    #   N · g = (I - J⁺J) · g = g - J⁺ · (J · g)
    # So: β = J · g_orient (3×1), then α = solve(A, β) (3×1), then J⁺β = J^T · α (6×1)

    # β = J · g_orient
    beta = [0.0] * 3
    for i in range(3):
        s = 0.0
        for j in range(6):
            s += J[i][j] * g_orient[j]
        beta[i] = s

    # α = A⁻¹ · β (solve A·α = β)
    alpha = _solve_3x3(A, beta)

    # J⁺β = J^T · α
    jt_alpha = [0.0] * 6
    for j in range(6):
        s = 0.0
        for i in range(3):
            s += J[i][j] * alpha[i]
        jt_alpha[j] = s

    # N · g = g - J⁺ · J · g = g - J⁺β = g - J^T·α
    null_grad = [g_orient[j] - jt_alpha[j] for j in range(6)]

    # Combine: Δθ_pos + scale * N·g
    scale = 0.08
    result = [delta_pos[j] + scale * null_grad[j] for j in range(6)]
    return result


def _nullspace_optimize_orientation(
    joint_angles: tuple[float, ...],
    target_xyz: tuple[float, float, float],
    config: Config,
    max_iters: int = 80,
) -> tuple[float, ...]:
    """Post-convergence null-space orientation refinement.

    After position IK converges, interleave null-space orientation steps
    with position corrections to maintain accuracy while pointing downward.
    """
    from src.control.fk_solver import _get_tool0_z_axis, _solve_fk_urdf_chain

    theta = list(joint_angles)

    for outer in range(20):
        # Orientation steps (5 at a time)
        for _ in range(5):
            _, _, zz = _get_tool0_z_axis(tuple(theta), config)
            if -zz > 0.96:
                break
            J = _compute_jacobian(tuple(theta), config)
            g_orient = _orientation_gradient(tuple(theta), config)
            delta_null = _nullspace_project(J, [0.0] * 6, g_orient, 0.08)
            for j in range(6):
                theta[j] += delta_null[j]
                lo, hi = _JOINT_LIMITS[j]
                theta[j] = max(lo, min(hi, theta[j]))

        # Position correction (correct any drift)
        current = _solve_fk_urdf_chain(tuple(theta), config)
        err_vec = [target_xyz[i] - current[i] for i in range(3)]
        err = math.sqrt(sum(e**2 for e in err_vec))
        if err > 0.002:
            J = _compute_jacobian(tuple(theta), config)
            delta_pos = _dls_step(J, err_vec, 0.08)
            for j in range(6):
                theta[j] += delta_pos[j]
                lo, hi = _JOINT_LIMITS[j]
                theta[j] = max(lo, min(hi, theta[j]))

        # Check combined convergence
        _, _, zz = _get_tool0_z_axis(tuple(theta), config)
        current = _solve_fk_urdf_chain(tuple(theta), config)
        pos_err = math.sqrt(sum((current[i] - target_xyz[i])**2 for i in range(3)))
        if -zz > 0.92 and pos_err < 0.004:
            break

    return tuple(round(_wrap_to_pi(t), 4) for t in theta)


def _frange(start: float, stop: float, step: float):
    """Float range generator, inclusive of stop."""
    v = start
    while v <= stop + step * 0.5:
        yield v
        v += step
