from __future__ import annotations

import math

from src.common.types import Obstacle
from src.planning.collision_checker import direct_path_clear


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
            if direct_path_clear(
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
