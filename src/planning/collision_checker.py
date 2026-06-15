from __future__ import annotations

import math

from src.common.types import CollisionCheckResult, Obstacle, ObstacleShape


def _obstacle_spans_z(obstacle: Obstacle, z: float) -> bool:
    """判断障碍物的垂直范围是否包含给定的 z 高度。

    用于 2D 碰撞检测的 z 轴过滤——只将与当前飞行高度有交集的
    障碍物纳入检测，避免低矮棋子阻挡高空 transfer 路径。
    """
    shape = obstacle.shape
    cz = obstacle.center_xyz[2]

    if shape == ObstacleShape.FLOATING_SPHERE:
        return abs(z - cz) <= obstacle.radius
    elif shape == ObstacleShape.FLOATING_CUBE:
        return abs(z - cz) <= obstacle.radius  # radius = 半边长（含 z 方向）
    elif shape == ObstacleShape.VERTICAL_CYLINDER:
        bottom = cz                      # 圆柱底面
        top = cz + obstacle.height       # 圆柱顶面
        return bottom <= z <= top
    else:
        # HORIZONTAL_CYLINDER / AABB / 未知：保守处理，始终纳入
        return True


def check_segment_collision(
    start_xyz: tuple[float, float, float],
    end_xyz: tuple[float, float, float],
    obstacles: list[Obstacle],
    step_size: float = 0.005,
    safety_margin: float = 0.0,
) -> CollisionCheckResult:
    """沿线段 start→end 采样检测是否与障碍物碰撞。

    对线段做均匀采样，每个采样点检查是否在任何障碍物的膨胀体积内。
    支持 VERTICAL_CYLINDER（投影为圆）和 HORIZONTAL_CYLINDER（投影为 AABB）。

    Args:
        start_xyz: 线段起点 (x, y, z)
        end_xyz: 线段终点 (x, y, z)
        obstacles: 障碍物列表
        step_size: 检测步长 (m)
        safety_margin: 额外安全膨胀距离 (m)

    Returns:
        CollisionCheckResult(collision_free, min_clearance, collision_point)
    """
    sx, sy, sz = start_xyz
    ex, ey, ez = end_xyz
    dx, dy, dz = ex - sx, ey - sy, ez - sz
    seg_length = math.hypot(dx, dy, dz)

    if seg_length < 1e-9:
        # 起点=终点：只检测该点
        return _check_point_vs_obstacles(sx, sy, obstacles, safety_margin)

    n_steps = max(2, int(math.ceil(seg_length / step_size)) + 1)
    min_clearance = float("inf")
    first_collision: tuple[float, float, float] | None = None

    for i in range(n_steps):
        t = i / (n_steps - 1)
        px = sx + t * dx
        py = sy + t * dy
        pz = sz + t * dz

        for obstacle in obstacles:
            # z 轴过滤：只检测与当前采样高度有交集的障碍物
            if not _obstacle_spans_z(obstacle, pz):
                continue
            clearance = _point_obstacle_clearance(px, py, obstacle)
            if clearance <= safety_margin:
                return CollisionCheckResult(
                    collision_free=False,
                    min_clearance=clearance,
                    collision_point=(px, py, pz),
                )
            if clearance < min_clearance:
                min_clearance = clearance

    return CollisionCheckResult(collision_free=True, min_clearance=min_clearance)


def direct_path_clear(
    start_xy: tuple[float, float],
    end_xy: tuple[float, float],
    z_plane: float,
    obstacles: list[Obstacle],
    step_size: float = 0.005,
    safety_margin: float = 0.0,
) -> bool:
    """检查 z_plane 高度上 start_xy→end_xy 直线是否无障碍。

    这是 check_segment_collision 在固定高度平面上的便捷封装。

    Args:
        start_xy: 起点 (x, y)
        end_xy: 终点 (x, y)
        z_plane: 检测平面高度 (m)
        obstacles: 障碍物列表
        step_size: 检测步长 (m)
        safety_margin: 额外安全膨胀距离 (m)

    Returns:
        True 如果整条线段无障碍
    """
    result = check_segment_collision(
        (start_xy[0], start_xy[1], z_plane),
        (end_xy[0], end_xy[1], z_plane),
        obstacles,
        step_size=step_size,
        safety_margin=safety_margin,
    )
    return result.collision_free


def check_segment_collision_multi_z(
    start_xy: tuple[float, float],
    end_xy: tuple[float, float],
    z_plane: float,
    obstacles: list[Obstacle],
    step_size: float = 0.005,
    safety_margin: float = 0.0,
) -> bool:
    """在多个高度层检测水平线段是否无障碍。

    对 z_plane, z_plane-0.05, z_plane-0.10 三个高度分别检测，
    只要任一层碰撞即判定为不可通行。这为机械臂中间连杆
    （肘部、前臂）提供了额外的安全余量。

    Args:
        start_xy: 起点 (x, y)
        end_xy: 终点 (x, y)
        z_plane: 主检测平面高度 (m)
        obstacles: 障碍物列表
        step_size: 检测步长 (m)
        safety_margin: 额外安全膨胀距离 (m)

    Returns:
        True 如果所有高度层均无障碍（整条线段可通行）
    """
    z_levels = [z_plane, z_plane - 0.05, z_plane - 0.10]
    for z in z_levels:
        if not direct_path_clear(start_xy, end_xy, z, obstacles, step_size, safety_margin):
            return False
    return True


# ── 内部辅助函数 ──


def _point_obstacle_clearance(
    px: float, py: float, obstacle: Obstacle
) -> float:
    """计算 2D 点到障碍物的 clearance（负值=穿透）。"""
    if obstacle.shape == ObstacleShape.HORIZONTAL_CYLINDER:
        return _point_horizontal_cylinder_clearance(px, py, obstacle)
    elif obstacle.shape == ObstacleShape.FLOATING_CUBE:
        return _point_aabb_clearance(px, py, obstacle)
    else:
        # VERTICAL_CYLINDER / FLOATING_SPHERE / AABB 及默认：投影为圆
        ox, oy = obstacle.center_xyz[0], obstacle.center_xyz[1]
        dist = math.hypot(px - ox, py - oy)
        return dist - obstacle.radius


def _point_aabb_clearance(px: float, py: float, obstacle: Obstacle) -> float:
    """计算 2D 点到轴对齐方形（浮空立方体 XY 投影）的 clearance。

    obstacle.radius 复用为半边长；立方体无旋转，AABB 为 [cx±r, cy±r]。
    """
    cx, cy = obstacle.center_xyz[0], obstacle.center_xyz[1]
    half = obstacle.radius  # 半边长
    # 点到 AABB 的最近距离
    dx = max(0.0, abs(px - cx) - half)
    dy = max(0.0, abs(py - cy) - half)
    dist = math.hypot(dx, dy)
    # 点在内部时返回负值
    if abs(px - cx) <= half and abs(py - cy) <= half:
        # 穿透深度 = 到最近边的距离（取负）
        penetration = half - max(abs(px - cx), abs(py - cy))
        return -penetration
    return dist


def _point_horizontal_cylinder_clearance(
    px: float, py: float, obstacle: Obstacle
) -> float:
    """计算 2D 点到水平圆柱（AABB 投影）的 clearance。

    水平圆柱沿其主轴方向延伸，在 XY 平面上投影为矩形。
    当前支持绕 Y 轴旋转（圆柱沿 X 轴）和绕 X 轴旋转（圆柱沿 Y 轴）。

    Returns:
        点与 AABB 的最近距离（负值=穿透）
    """
    cx, cy = obstacle.center_xyz[0], obstacle.center_xyz[1]
    roll, pitch, yaw = obstacle.orientation_rpy

    # 根据旋转确定圆柱主轴方向
    if abs(pitch - math.pi / 2) < 0.01 or abs(pitch + math.pi / 2) < 0.01:
        # 绕 Y 轴旋转 ±90°：圆柱沿 X 轴
        half_length = obstacle.height / 2.0  # 沿 X
        half_width = obstacle.radius           # 沿 Y
    elif abs(roll - math.pi / 2) < 0.01 or abs(roll + math.pi / 2) < 0.01:
        # 绕 X 轴旋转 ±90°：圆柱沿 Y 轴
        half_length = obstacle.height / 2.0   # 沿 Y
        half_width = obstacle.radius           # 沿 X
    else:
        # 一般情况：计算主轴在 XY 平面的投影方向
        # 简化处理：使用圆柱的包围圆（保守估计）
        half_diag = math.hypot(obstacle.height / 2.0, obstacle.radius)
        dist = math.hypot(px - cx, py - cy)
        return dist - half_diag

    if abs(pitch - math.pi / 2) < 0.01 or abs(pitch + math.pi / 2) < 0.01:
        # X 轴延伸
        closest_x = max(cx - half_length, min(cx + half_length, px))
        closest_y = max(cy - half_width, min(cy + half_width, py))
    else:
        # Y 轴延伸
        closest_x = max(cx - half_width, min(cx + half_width, px))
        closest_y = max(cy - half_length, min(cy + half_length, py))

    dist = math.hypot(px - closest_x, py - closest_y)
    return dist


def _check_point_vs_obstacles(
    px: float, py: float, obstacles: list[Obstacle], safety_margin: float
) -> CollisionCheckResult:
    """检测单个 2D 点 vs 障碍物列表（用于线段长度为零的情况）。"""
    min_clearance = float("inf")
    for obstacle in obstacles:
        clearance = _point_obstacle_clearance(px, py, obstacle)
        if clearance <= safety_margin:
            return CollisionCheckResult(
                collision_free=False,
                min_clearance=clearance,
                collision_point=(px, py, 0.0),
            )
        if clearance < min_clearance:
            min_clearance = clearance
    return CollisionCheckResult(collision_free=True, min_clearance=min_clearance)
