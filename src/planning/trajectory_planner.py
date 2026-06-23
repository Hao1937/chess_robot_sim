from __future__ import annotations

import math

from src.common.config import DEFAULT_CONFIG, Config
from src.common.types import JointTrajectory, MotionPrimitive, Obstacle, PrimitivePlanningContext
from src.planning.collision_checker import direct_path_clear
from src.planning.ik_solver import current_joint_seed, is_reachable, solve_ik
from src.planning.path_search import a_star_2d
from src.planning.trajectory_smoother import (
    interpolate_waypoints_cartesian,
    shortcut_smoothing,
    smooth_joint_trajectory,
)
from src.planning.visualization import draw_direct_line, draw_path_debug


def plan_trajectory(
    primitives_or_contexts: list[MotionPrimitive] | list[PrimitivePlanningContext],
    obstacles: list[Obstacle] | None = None,
    config: Config = DEFAULT_CONFIG,
    *,
    enable_path_search: bool = True,
    enable_smoothing: bool = True,
    enable_interpolation: bool = True,
    start_xyz: tuple[float, float, float] | None = None,
) -> JointTrajectory:
    """规划关节轨迹。

    对水平移动 (approach/transfer) 可启用路径搜索绕障，
    对垂直移动保持现有 IK 逐点求解。
    新增 enable_* 参数控制各功能的开关，方便测试和渐进式集成。

    Args:
        primitives_or_contexts: MotionPrimitive 或 PrimitivePlanningContext 列表
        obstacles: 全局障碍物列表（当传入 MotionPrimitive 时使用）
        config: 配置对象
        enable_path_search: 是否启用 A* 路径搜索绕障
        enable_smoothing: 是否启用 shortcut + joint 平滑
        enable_interpolation: 是否启用 waypoint 插值
        start_xyz: 起始 EE 世界坐标。传入时 prepend 为第一个 Cartesian waypoint，
            使第一个 approach/transfer primitive 能从此位置插值到目标。
            也用于绘制完整的规划轨迹线（从机器人实际位置开始）。

    Returns:
        JointTrajectory(joint_waypoints, speed_profile)
    """
    cartesian_waypoints: list[tuple[float, float, float]] = []
    # 预置起始点：让第一个 primitive 有 prev 可插值，规划轨迹从实际位置起算
    if start_xyz is not None:
        cartesian_waypoints.append(start_xyz)
    speed_profile: list[str] = []
    primitive_ranges: list[tuple[int, int]] = []

    for item in primitives_or_contexts:
        if isinstance(item, PrimitivePlanningContext):
            primitive = item.primitive
            primitive_obstacles = item.obstacles
        else:
            primitive = item
            primitive_obstacles = obstacles or []

        wp_before = len(cartesian_waypoints)

        if primitive.primitive_type in ("approach", "transfer"):
            _plan_horizontal_segment(
                primitive, primitive_obstacles, config,
                cartesian_waypoints, speed_profile,
                enable_path_search, enable_smoothing, enable_interpolation,
            )
        else:
            _plan_vertical_segment(
                primitive,
                cartesian_waypoints, speed_profile,
                enable_interpolation, config,
            )

        wp_after = len(cartesian_waypoints)
        primitive_ranges.append((wp_before, wp_after))

    # ── 确保 speed_profile 与 waypoints 等长 ──
    while len(speed_profile) < len(cartesian_waypoints):
        speed_profile.append("safe")
    speed_profile = speed_profile[:len(cartesian_waypoints)]

    # ── IK 转换（链式：前一个解作为下一个的种子，确保解分支连续）──
    # 从机器人当前实际关节角播种，确保轨迹与物理位置连续
    joint_waypoints: list[tuple[float, ...]] = []
    seed = current_joint_seed(config)
    for wp in cartesian_waypoints:
        jw = solve_ik(wp, config, seed=seed)
        joint_waypoints.append(jw)
        seed = jw

    # ── 关节空间平滑 ──
    # 链式 IK 种子的使用使得邻近 waypoint 的关节配置连续，
    # 平滑不再产生物理无意义的混合。
    if enable_smoothing and len(joint_waypoints) >= 3:
        joint_waypoints = smooth_joint_trajectory(joint_waypoints)

    return JointTrajectory(
        joint_waypoints=joint_waypoints,
        speed_profile=speed_profile[:len(joint_waypoints)],
        primitive_ranges=primitive_ranges,
    )


# ── 内部辅助函数 ──


def _plan_horizontal_segment(
    primitive: MotionPrimitive,
    primitive_obstacles: list[Obstacle],
    config: Config,
    cartesian_waypoints: list[tuple[float, float, float]],
    speed_profile: list[str],
    enable_path_search: bool,
    enable_smoothing: bool,
    enable_interpolation: bool,
) -> None:
    """规划水平移动段 (approach/transfer) 的路径。

    如果直线路径被阻挡且 enable_path_search=True，使用 A* 绕行；
    否则直接走直线（可插值）。
    """
    end_xyz = primitive.target_xyz
    z_plane = end_xyz[2]
    prev = cartesian_waypoints[-1] if cartesian_waypoints else None

    start_xy = (prev[0], prev[1]) if prev else (end_xyz[0], end_xyz[1])
    end_xy = (end_xyz[0], end_xyz[1])

    need_detour = (
        enable_path_search
        and prev is not None
        and not direct_path_clear(
            start_xy, end_xy, z_plane, primitive_obstacles,
            step_size=config.path_collision_check_step,
            safety_margin=config.safety_margin,
        )
    )

    if need_detour:
        # ── 可视化：画出直接路径（绿色）以示对比 ──
        draw_direct_line(
            (start_xy[0], start_xy[1], z_plane),
            (end_xy[0], end_xy[1], z_plane),
        )

        search_result = a_star_2d(
            start_xy, end_xy,
            obstacles=primitive_obstacles,
            z_plane=z_plane,
            grid_resolution=config.path_grid_resolution,
            timeout_ms=config.path_search_timeout_ms,
            config=config,
        )

        if search_result.success and len(search_result.path_xy) > 0:
            path_3d: list[tuple[float, float, float]] = [
                (x, y, z_plane) for x, y in search_result.path_xy
            ]

            if enable_smoothing:
                path_3d = shortcut_smoothing(
                    search_result.path_xy, z_plane, primitive_obstacles,
                    collision_check_step=config.path_collision_check_step,
                    safety_margin=config.safety_margin,
                )

            if enable_interpolation and len(path_3d) >= 2:
                path_3d = interpolate_waypoints_cartesian(
                    path_3d, config.waypoint_interpolation_step,
                )

            # ── 可视化：画出 A* 绕行路径（红色） ──
            draw_path_debug(path_3d, color=(0.9, 0.15, 0.1))

            # 跳过首点（与 prev 重复）
            start_idx = 1 if prev is not None else 0
            new_points = path_3d[start_idx:]
            cartesian_waypoints.extend(new_points)
            speed_profile.extend(["safe"] * len(new_points))
            return

        # A* 失败 → 尝试 3D 飞越障碍物
        if _try_overfly_horizontal(
            (start_xy[0], start_xy[1], z_plane),
            end_xyz,
            primitive_obstacles,
            config,
            cartesian_waypoints,
            speed_profile,
            enable_interpolation,
        ):
            return

    # 直接路径（无障碍或 overfly 失败 fallback）
    _append_with_interpolation(
        prev, end_xyz,
        cartesian_waypoints, speed_profile,
        enable_interpolation, config.waypoint_interpolation_step,
        speed_mode="fast",
    )


def _plan_vertical_segment(
    primitive: MotionPrimitive,
    cartesian_waypoints: list[tuple[float, float, float]],
    speed_profile: list[str],
    enable_interpolation: bool,
    config: Config,
) -> None:
    """规划垂直移动段 (descend/lift/grasp/detach/retreat/pause) 的路径。"""
    end_xyz = primitive.target_xyz
    prev = cartesian_waypoints[-1] if cartesian_waypoints else None

    _append_with_interpolation(
        prev, end_xyz,
        cartesian_waypoints, speed_profile,
        enable_interpolation, config.waypoint_vertical_step,
        speed_mode="safe",
    )


def _append_with_interpolation(
    prev: tuple[float, float, float] | None,
    target: tuple[float, float, float],
    cartesian_waypoints: list[tuple[float, float, float]],
    speed_profile: list[str],
    enable_interpolation: bool,
    step_size: float,
    speed_mode: str,
) -> None:
    """将 target 添加到 waypoint 列表，可选沿 prev→target 插值。"""
    if enable_interpolation and prev is not None:
        segment = interpolate_waypoints_cartesian(
            [prev, target], step_size,
        )
        if segment and len(segment) > 1:
            new_points = segment[1:]  # 跳过首点（= prev）
            cartesian_waypoints.extend(new_points)
            speed_profile.extend([speed_mode] * len(new_points))
            return

    # 无法插值或第一个点：直接添加
    cartesian_waypoints.append(target)
    speed_profile.append(speed_mode)

def _try_overfly_horizontal(
    start_xyz: tuple[float, float, float],
    end_xyz: tuple[float, float, float],
    obstacles: list[Obstacle],
    config: Config,
    cartesian_waypoints: list,
    speed_profile: list,
    enable_interpolation: bool,
) -> bool:
    """Attempt 3D overfly when 2D A* fails.

    Finds the tallest obstacle blocking the direct horizontal path,
    calculates a safe overfly height above it, and adds rise/transfer/
    descent waypoints.

    Returns True if overfly succeeded (waypoints were added).
    """
    # Use the actual last waypoint as the rise start point
    prev = cartesian_waypoints[-1] if cartesian_waypoints else None
    if prev is None:
        return False

    start_xy = (prev[0], prev[1])
    end_xy = (end_xyz[0], end_xyz[1])

    # Sample the direct horizontal line for max blocking obstacle height
    dx = end_xy[0] - start_xy[0]
    dy = end_xy[1] - start_xy[1]
    seg_length = math.hypot(dx, dy)

    max_obstacle_height = 0.0
    if seg_length < 1e-9:
        px, py = start_xy
        for obstacle in obstacles:
            ox, oy = obstacle.center_xyz[0], obstacle.center_xyz[1]
            dist = math.hypot(px - ox, py - oy)
            if dist < obstacle.radius + config.safety_margin:
                if obstacle.height > max_obstacle_height:
                    max_obstacle_height = obstacle.height
    else:
        n_steps = max(2, int(math.ceil(seg_length / config.path_collision_check_step)) + 1)
        for i in range(n_steps):
            t = i / (n_steps - 1)
            px = start_xy[0] + t * dx
            py = start_xy[1] + t * dy
            for obstacle in obstacles:
                ox, oy = obstacle.center_xyz[0], obstacle.center_xyz[1]
                dist = math.hypot(px - ox, py - oy)
                if dist < obstacle.radius + config.safety_margin:
                    if obstacle.height > max_obstacle_height:
                        max_obstacle_height = obstacle.height

    # Calculate overfly height
    overfly_z = max_obstacle_height + config.obstacle_overfly_clearance
    # Never fly lower than the target plane (prevents descending overfly)
    if overfly_z <= end_xyz[2]:
        return False

    # Check reachability at overfly height
    if not is_reachable((end_xy[0], end_xy[1], overfly_z), config):
        return False

    # Filter obstacles: only those taller than overfly_z matter at this height
    filtered_obstacles = [
        o for o in obstacles
        if o.height >= overfly_z
    ]

    # Check horizontal clearance at overfly_z
    if not direct_path_clear(
        start_xy, end_xy, overfly_z, filtered_obstacles,
        step_size=config.path_collision_check_step,
        safety_margin=config.safety_margin,
    ):
        return False

    # Add overfly waypoints: rise to overfly_z
    rise_target = (prev[0], prev[1], overfly_z)
    _append_with_interpolation(
        prev, rise_target,
        cartesian_waypoints, speed_profile,
        enable_interpolation, config.waypoint_vertical_step,
        speed_mode="safe",
    )

    # Horizontal transfer at overfly_z
    actual_prev = cartesian_waypoints[-1]
    transfer_target = (end_xy[0], end_xy[1], overfly_z)
    _append_with_interpolation(
        actual_prev, transfer_target,
        cartesian_waypoints, speed_profile,
        enable_interpolation, config.waypoint_interpolation_step,
        speed_mode="safe",
    )

    # Descent to original target
    actual_prev = cartesian_waypoints[-1]
    _append_with_interpolation(
        actual_prev, end_xyz,
        cartesian_waypoints, speed_profile,
        enable_interpolation, config.waypoint_vertical_step,
        speed_mode="safe",
    )

    return True
