from __future__ import annotations

import math

from src.common.types import Obstacle
from src.planning.collision_checker import direct_path_clear, check_segment_collision_multi_z


def interpolate_waypoints_cartesian(
    path_xyz: list[tuple[float, float, float]],
    step_size: float = 0.03,
) -> list[tuple[float, float, float]]:
    """在 Cartesian 空间对路径点做线性插值。

    对相邻路径点之间的线段做均匀采样，使得输出 waypoint 间距 ≤ step_size。
    保留首尾端点不变。

    Args:
        path_xyz: 原始 3D 路径点序列
        step_size: 目标插值步长 (m)

    Returns:
        密集插值后的路径点列表
    """
    if len(path_xyz) < 2:
        return list(path_xyz)

    result: list[tuple[float, float, float]] = [path_xyz[0]]

    for i in range(len(path_xyz) - 1):
        p0 = path_xyz[i]
        p1 = path_xyz[i + 1]
        dx = p1[0] - p0[0]
        dy = p1[1] - p0[1]
        dz = p1[2] - p0[2]
        seg_len = math.hypot(dx, dy, dz)

        if seg_len < 1e-9:
            continue

        n_steps = max(1, int(math.ceil(seg_len / step_size)))
        # 跳过 p0（已在结果中），从 1 开始
        for j in range(1, n_steps + 1):
            t = j / n_steps
            result.append((
                p0[0] + t * dx,
                p0[1] + t * dy,
                p0[2] + t * dz,
            ))

    return result


def shortcut_smoothing(
    path_xy: list[tuple[float, float]],
    z_plane: float,
    obstacles: list[Obstacle],
    collision_check_step: float = 0.005,
    safety_margin: float = 0.0,
) -> list[tuple[float, float, float]]:
    """对 2D 路径应用 shortcut 算法：贪心跳过冗余 waypoint。

    算法：从 path[0] 开始，尝试跳过尽可能多的中间点，
    如果能从当前点直接连线到 path[j] 且无障碍，就跳过 i+1..j-1。
    贪心策略选择最远的可行跳跃。

    Args:
        path_xy: A* 输出的 2D 路径点 [(x, y), ...]
        z_plane: 检测平面高度 (m)
        obstacles: 障碍物列表
        collision_check_step: 碰撞检测步长 (m)
        safety_margin: 额外安全膨胀 (m)

    Returns:
        平滑后的 3D 路径点 [(x, y, z_plane), ...]
    """
    if len(path_xy) < 3:
        return [(x, y, z_plane) for x, y in path_xy]

    smoothed_xy: list[tuple[float, float]] = [path_xy[0]]
    i = 0

    while i < len(path_xy) - 1:
        # 从最远端向前尝试，找最长的可行跳跃
        jumped = False
        for j in range(len(path_xy) - 1, i + 1, -1):
            if check_segment_collision_multi_z(
                path_xy[i], path_xy[j], z_plane,
                obstacles,
                step_size=collision_check_step,
                safety_margin=safety_margin,
            ):
                smoothed_xy.append(path_xy[j])
                i = j
                jumped = True
                break

        if not jumped:
            # 无可行跳跃，前进一格
            i += 1
            smoothed_xy.append(path_xy[i])

    return [(x, y, z_plane) for x, y in smoothed_xy]


def smooth_joint_trajectory(
    joint_waypoints: list[tuple[float, ...]],
    smoothing_window: int = 3,
) -> list[tuple[float, ...]]:
    """对关节空间 waypoint 做移动平均平滑，减少 jerk。

    对每个关节维度独立做 1D 移动平均，边界使用较小窗口。
    如果 waypoint 数量不足以执行平滑，直接返回原始序列。

    Args:
        joint_waypoints: 原始关节角度序列 [(j0, j1, ..., j5), ...]
        smoothing_window: 移动平均窗口大小（奇数，默认 3）

    Returns:
        平滑后的关节角度序列
    """
    n = len(joint_waypoints)
    if n < 3 or smoothing_window < 2:
        return list(joint_waypoints)

    num_joints = len(joint_waypoints[0])
    half = smoothing_window // 2
    smoothed: list[list[float]] = [[] for _ in range(num_joints)]

    for joint_idx in range(num_joints):
        values = [wp[joint_idx] for wp in joint_waypoints]
        smoothed_values: list[float] = []

        for k in range(n):
            # 边界自适应：两端使用较小窗口
            left = max(0, k - half)
            right = min(n - 1, k + half)
            window = values[left:right + 1]
            avg = sum(window) / len(window)
            smoothed_values.append(avg)

        smoothed[joint_idx] = smoothed_values

    # 转置回 waypoint 序列
    return [
        tuple(smoothed[joint_idx][k] for joint_idx in range(num_joints))
        for k in range(n)
    ]


# ── Cubic Spline smoothing (P5b) ──


def _solve_tridiagonal(a: list[float], b: list[float], c: list[float], d: list[float]) -> list[float]:
    """Thomas algorithm for tridiagonal linear system.

    Solves:
      a[i]*x[i-1] + b[i]*x[i] + c[i]*x[i+1] = d[i]   for i = 0..n-1
    with a[0] = 0 and c[n-1] = 0.

    All four lists must have the same length n.
    """
    n = len(b)
    # Work on copies so the caller's arrays are not mutated
    c_prime = c[:]
    d_prime = d[:]
    x = [0.0] * n

    # Forward sweep
    c_prime[0] = c[0] / b[0] if abs(b[0]) > 1e-12 else 0.0
    d_prime[0] = d[0] / b[0] if abs(b[0]) > 1e-12 else 0.0
    for i in range(1, n):
        denom = b[i] - a[i] * c_prime[i - 1]
        if abs(denom) > 1e-12:
            if i < n - 1:
                c_prime[i] = c[i] / denom
            d_prime[i] = (d[i] - a[i] * d_prime[i - 1]) / denom
        else:
            c_prime[i] = 0.0
            d_prime[i] = 0.0

    # Back substitution
    x[-1] = d_prime[-1]
    for i in range(n - 2, -1, -1):
        x[i] = d_prime[i] - c_prime[i] * x[i + 1]

    return x


def smooth_joint_trajectory_cubic_spline(
    joint_waypoints: list[tuple[float, ...]],
    num_samples: int | None = None,
) -> list[tuple[float, ...]]:
    """Fit a natural cubic spline to joint waypoints and resample.

    Each joint dimension is independently fitted with a C^2 continuous
    piecewise cubic polynomial.  Waypoint indices (0, 1, ..., n-1) serve
    as the independent parameter, and the spline is uniformly resampled.

    This is a pure-Python implementation that does **not** require scipy.

    Args:
        joint_waypoints: original joint waypoints [(j0,...,j5), ...]
        num_samples: number of output samples (default: same as input)

    Returns:
        C^2 continuous joint trajectory of length num_samples
    """
    n = len(joint_waypoints)
    if n < 3:
        return list(joint_waypoints)

    if num_samples is None:
        num_samples = n

    num_joints = len(joint_waypoints[0])

    # t_i = i (waypoint index as parameter)
    t = list(range(n))

    result: list[list[float]] = [[] for _ in range(num_joints)]

    for joint_idx in range(num_joints):
        y = [wp[joint_idx] for wp in joint_waypoints]

        # ── Build natural cubic spline ──
        n_seg = n - 1
        h = [t[i + 1] - t[i] for i in range(n_seg)]

        # Tridiagonal system for second derivatives m_i (i = 0..n-1)
        # Natural boundary: m_0 = m_{n-1} = 0
        m = [0.0] * n

        if n > 2:
            # RHS = 6 * (dy_i - dy_{i-1})  where dy_i = (y_{i+1} - y_i) / h_i
            dy = [(y[i + 1] - y[i]) / h[i] if h[i] > 0 else 0.0 for i in range(n_seg)]

            # System for interior points i = 1..n-2:
            # h_{i-1}*m_{i-1} + 2*(h_{i-1}+h_i)*m_i + h_i*m_{i+1} = 6*(dy_i - dy_{i-1})
            size = n - 2
            if size > 0:
                a_tri = [0.0] * size
                b_tri = [0.0] * size
                c_tri = [0.0] * size
                d_tri = [0.0] * size

                for idx in range(size):
                    i = idx + 1  # actual index in [1..n-2]
                    a_tri[idx] = h[i - 1]
                    b_tri[idx] = 2.0 * (h[i - 1] + h[i])
                    c_tri[idx] = h[i]
                    d_tri[idx] = 6.0 * (dy[i] - dy[i - 1])

                # Solve tridiagonal
                m_interior = _solve_tridiagonal(a_tri, b_tri, c_tri, d_tri)
                for idx in range(size):
                    m[idx + 1] = m_interior[idx]

        # ── Compute spline coefficients for each segment ──
        # Segment i: S_i(t) = a_i + b_i*(t-t_i) + c_i*(t-t_i)^2 + d_i*(t-t_i)^3
        # t in [t_i, t_{i+1}]
        a_coeff = [0.0] * n_seg
        b_coeff = [0.0] * n_seg
        c_coeff = [0.0] * n_seg
        d_coeff = [0.0] * n_seg

        for i in range(n_seg):
            hi = h[i]
            if hi < 1e-12:
                a_coeff[i] = y[i]
                b_coeff[i] = 0.0
                c_coeff[i] = 0.0
                d_coeff[i] = 0.0
                continue
            a_coeff[i] = y[i]
            c_coeff[i] = m[i] / 2.0
            d_coeff[i] = (m[i + 1] - m[i]) / (6.0 * hi)
            b_coeff[i] = (y[i + 1] - y[i]) / hi - hi * (2.0 * m[i] + m[i + 1]) / 6.0

        # ── Uniform resampling in [0, n-1] ──
        smp = []
        t_param = [float(i) for i in range(n)]
        for k in range(num_samples):
            u = t_param[0] + (t_param[-1] - t_param[0]) * k / max(1, num_samples - 1)

            # Find segment
            seg_idx = 0
            for i_seg in range(n_seg):
                if u <= t_param[i_seg + 1] + 1e-10:
                    seg_idx = i_seg
                    break
            else:
                seg_idx = n_seg - 1

            dt_val = u - t_param[seg_idx]
            val = (
                a_coeff[seg_idx]
                + b_coeff[seg_idx] * dt_val
                + c_coeff[seg_idx] * dt_val * dt_val
                + d_coeff[seg_idx] * dt_val * dt_val * dt_val
            )
            smp.append(val)

        result[joint_idx] = smp

    # Transpose back to waypoint tuples
    return [
        tuple(result[joint_idx][k] for joint_idx in range(num_joints))
        for k in range(num_samples)
    ]


# ── Jerk-optimal trajectory optimization (P5c) ──


def _compute_jerk_cost(
    waypoints: list[list[float]],
    dt_values: list[float],
) -> float:
    """Compute total squared-jerk cost for a joint trajectory.

    Jerk at waypoint i: j_i = (a_{i+1} - a_i) / dt_avg
    where a_i = (v_{i+1} - v_i) / dt_avg
    and   v_i = (theta_{i+1} - theta_i) / dt_i

    Cost = sum over all joints of sum_i ||j_i||^2
    """
    num_pts = len(waypoints)
    if num_pts < 4:
        return 0.0
    num_joints = len(waypoints[0])

    # Velocity at edge i (between waypoint i and i+1): v_i
    v: list[list[float]] = []
    for i in range(num_pts - 1):
        dt = max(dt_values[i], 1e-9)
        v.append([(waypoints[i + 1][j] - waypoints[i][j]) / dt for j in range(num_joints)])

    # Acceleration at waypoint i (i = 1..num_pts-2): a_i
    a: list[list[float]] = []
    for i in range(1, num_pts - 1):
        dt_avg = max((dt_values[i - 1] + dt_values[i]) / 2.0, 1e-9)
        a.append([(v[i][j] - v[i - 1][j]) / dt_avg for j in range(num_joints)])

    # Jerk at waypoint i (i = 2..num_pts-3): j_i
    cost = 0.0
    for i in range(2, num_pts - 2):
        dt_avg = max((dt_values[i - 1] + dt_values[i]) / 2.0, 1e-9)
        for j in range(num_joints):
            jerk_val = (a[i - 1][j] - a[i - 2][j]) / dt_avg  # a indexed from i-2
            cost += jerk_val * jerk_val

    return cost


def optimize_jerk_minimum(
    joint_waypoints: list[tuple[float, ...]],
    speed_profile: list[str],
    num_iterations: int = 100,
    learning_rate: float = 0.01,
) -> list[tuple[float, ...]]:
    """Iterative jerk minimization via gradient descent.

    Minimizes the integral of squared jerk (third derivative) along the
    trajectory.  Uses finite-difference gradients and a simple fixed-step
    descent.  Endpoints (first / last waypoint) are kept fixed.

    The speed_profile entries (\"fast\" / \"safe\") are mapped to
    approximate time steps so that the optimisation respects the
    per-segment speed mode.

    This is a pure-Python implementation with **no** external dependencies.

    .. warning::
       **KNOWN BUG (2026-06-14):** 有限差分梯度对3阶导数代价函数数值不稳定。
       epsilon=1e-6太小，导致梯度被放大约10^14倍，更新后关节值变为天文数字。
       修复方向：使用分析梯度、更大epsilon(0.01)、梯度裁剪、或更小学习率(1e-14)。
       当前通过 plan_trajectory(enable_jerk_opt=False) 默认禁用。

    Args:
        joint_waypoints: original joint waypoints [(j0,...,j5), ...]
        speed_profile: per-waypoint speed mode; length must match
        num_iterations: number of gradient-descent iterations
        learning_rate: step size for each gradient update

    Returns:
        Jerk-minimised joint trajectory (same length as input)
    """
    n = len(joint_waypoints)
    if n < 4:
        return list(joint_waypoints)

    num_joints = len(joint_waypoints[0])
    if len(speed_profile) < n:
        # Pad speed_profile if shorter (should not happen in practice)
        sp = list(speed_profile) + ["safe"] * (n - len(speed_profile))
    else:
        sp = list(speed_profile[:n])

    # Map speed modes to approximate time steps
    _FAST_DT = 0.04
    _SAFE_DT = 0.08
    dt_values = [_FAST_DT if mode == "fast" else _SAFE_DT for mode in sp]

    # Mutable copy of waypoints
    wp = [list(wp_tuple) for wp_tuple in joint_waypoints]
    eps = 1e-6

    for _iteration in range(num_iterations):
        for i in range(1, n - 1):  # skip fixed endpoints
            for j in range(num_joints):
                orig = wp[i][j]

                # --- Finite-difference gradient ---
                wp[i][j] = orig + eps
                cost_plus = _compute_jerk_cost(wp, dt_values)

                wp[i][j] = orig - eps
                cost_minus = _compute_jerk_cost(wp, dt_values)

                grad = (cost_plus - cost_minus) / (2.0 * eps)

                # Gradient descent update
                wp[i][j] = orig - learning_rate * grad

    return [tuple(wp_tuple) for wp_tuple in wp]
