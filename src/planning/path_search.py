from __future__ import annotations

import heapq
import math
import time

from src.common.config import DEFAULT_CONFIG, Config
from src.common.types import Obstacle, ObstacleShape, PathSearchResult
from src.planning.collision_checker import direct_path_clear, check_segment_collision_multi_z


# A* 8-邻域方向：(dx, dy, cost)
_NEIGHBORS_8 = [
    (-1, -1, math.sqrt(2)), (0, -1, 1.0), (1, -1, math.sqrt(2)),
    (-1,  0, 1.0),                              (1,  0, 1.0),
    (-1,  1, math.sqrt(2)), (0,  1, 1.0), (1,  1, math.sqrt(2)),
]


def build_2d_occupancy_grid(
    obstacles: list[Obstacle],
    z_plane: float,
    resolution: float,
    bounds: tuple[float, float, float, float],
    safety_margin: float = 0.0,
) -> list[list[bool]]:
    """将 3D 障碍物投影到指定高度平面，生成 2D 占据栅格。

    根据障碍物形状分派标记逻辑：
    - VERTICAL_CYLINDER → 圆形区域
    - HORIZONTAL_CYLINDER → AABB 矩形区域

    Args:
        obstacles: 障碍物列表
        z_plane: 投影平面高度 (m)
        resolution: 网格分辨率 (m)
        bounds: (xmin, xmax, ymin, ymax) 搜索边界
        safety_margin: 障碍物膨胀距离 (m)

    Returns:
        2D 布尔网格，True=被占据（不可通行）
    """
    xmin, xmax, ymin, ymax = bounds
    cols = max(1, int(math.ceil((xmax - xmin) / resolution)))
    rows = max(1, int(math.ceil((ymax - ymin) / resolution)))
    grid = [[False] * cols for _ in range(rows)]

    for obstacle in obstacles:
        _mark_obstacle_on_grid(grid, obstacle, bounds, resolution, safety_margin)

    return grid


def a_star_2d(
    start_xy: tuple[float, float],
    end_xy: tuple[float, float],
    obstacles: list[Obstacle],
    z_plane: float = 0.18,
    grid_resolution: float = 0.02,
    timeout_ms: float = 100.0,
    bounds: tuple[float, float, float, float] | None = None,
    config: Config = DEFAULT_CONFIG,
) -> PathSearchResult:
    """在 z_plane 高度上做 2D A* 搜索，返回绕障路径。

    使用 8-邻域连接，Euclidean distance heuristic。
    搜索空间受 bounds 约束，边界格子一律视为 occupied。

    Args:
        start_xy: 起点 (x, y)
        end_xy: 终点 (x, y)
        obstacles: 障碍物列表
        z_plane: 搜索平面高度 (m)
        grid_resolution: 网格分辨率 (m)
        timeout_ms: 搜索超时 (ms)
        bounds: (xmin, xmax, ymin, ymax)，默认使用工作空间范围
        config: 配置对象

    Returns:
        PathSearchResult(success, path_xy, search_time_ms, nodes_explored)
    """
    t_start = time.perf_counter()

    if bounds is None:
        bounds = _default_search_bounds(config)

    xmin, xmax, ymin, ymax = bounds
    cols = max(1, int(math.ceil((xmax - xmin) / grid_resolution)))
    rows = max(1, int(math.ceil((ymax - ymin) / grid_resolution)))

    grid = build_2d_occupancy_grid(
        obstacles, z_plane, grid_resolution, bounds,
        safety_margin=config.safety_margin,
    )

    # 世界坐标 ↔ 网格索引
    def world_to_grid(x: float, y: float) -> tuple[int, int]:
        gx = int((x - xmin) / grid_resolution)
        gy = int((y - ymin) / grid_resolution)
        return (max(0, min(cols - 1, gx)), max(0, min(rows - 1, gy)))

    def grid_to_world(gx: int, gy: int) -> tuple[float, float]:
        return (xmin + (gx + 0.5) * grid_resolution, ymin + (gy + 0.5) * grid_resolution)

    start_gx, start_gy = world_to_grid(*start_xy)
    end_gx, end_gy = world_to_grid(*end_xy)

    # 起点/终点在障碍物内 → 向外搜索最近可通行格
    if grid[start_gy][start_gx]:
        start_gx, start_gy = _find_nearest_free(grid, start_gx, start_gy, cols, rows)
        if start_gx < 0:
            return PathSearchResult(success=False, search_time_ms=(time.perf_counter() - t_start) * 1000)

    if grid[end_gy][end_gx]:
        end_gx, end_gy = _find_nearest_free(grid, end_gx, end_gy, cols, rows)
        if end_gx < 0:
            return PathSearchResult(success=False, search_time_ms=(time.perf_counter() - t_start) * 1000)

    if (start_gx, start_gy) == (end_gx, end_gy):
        return PathSearchResult(
            success=True,
            path_xy=[grid_to_world(start_gx, start_gy)],
            search_time_ms=(time.perf_counter() - t_start) * 1000,
            nodes_explored=0,
        )

    # A* 搜索核心
    start_key = (start_gx, start_gy)
    end_key = (end_gx, end_gy)

    open_set: list[tuple[float, int, tuple[int, int]]] = []
    tie_breaker = 0
    h_start = _heuristic(start_gx, start_gy, end_gx, end_gy)
    heapq.heappush(open_set, (h_start, tie_breaker, start_key))
    tie_breaker += 1

    g_score: dict[tuple[int, int], float] = {start_key: 0.0}
    came_from: dict[tuple[int, int], tuple[int, int]] = {}
    closed_set: set[tuple[int, int]] = set()
    nodes_explored = 0
    timeout_sec = timeout_ms / 1000.0

    while open_set:
        if time.perf_counter() - t_start > timeout_sec:
            return PathSearchResult(
                success=False,
                search_time_ms=(time.perf_counter() - t_start) * 1000,
                nodes_explored=nodes_explored,
            )

        _, _, current = heapq.heappop(open_set)
        if current in closed_set:
            continue

        nodes_explored += 1

        if current == end_key:
            path_xy = _reconstruct_path(came_from, current, grid_to_world)
            return PathSearchResult(
                success=True,
                path_xy=path_xy,
                search_time_ms=(time.perf_counter() - t_start) * 1000,
                nodes_explored=nodes_explored,
            )

        closed_set.add(current)
        cx, cy = current

        for dx, dy, move_cost in _NEIGHBORS_8:
            nx, ny = cx + dx, cy + dy
            if nx < 0 or nx >= cols or ny < 0 or ny >= rows:
                continue
            if grid[ny][nx]:
                continue
            neighbor = (nx, ny)
            if neighbor in closed_set:
                continue

            tentative_g = g_score[current] + move_cost * grid_resolution
            if tentative_g < g_score.get(neighbor, float("inf")):
                g_score[neighbor] = tentative_g
                came_from[neighbor] = current
                f = tentative_g + _heuristic(nx, ny, end_gx, end_gy)
                heapq.heappush(open_set, (f, tie_breaker, neighbor))
                tie_breaker += 1

    return PathSearchResult(
        success=False,
        search_time_ms=(time.perf_counter() - t_start) * 1000,
        nodes_explored=nodes_explored,
    )


# ── 内部辅助函数 ──


def _heuristic(gx: int, gy: int, end_gx: int, end_gy: int) -> float:
    """A* heuristic: Euclidean 距离（admissible for 8-connected grid）。"""
    return math.hypot(end_gx - gx, end_gy - gy)


def _reconstruct_path(
    came_from: dict[tuple[int, int], tuple[int, int]],
    current: tuple[int, int],
    grid_to_world,
) -> list[tuple[float, float]]:
    """回溯重建路径。"""
    path: list[tuple[int, int]] = [current]
    while current in came_from:
        current = came_from[current]
        path.append(current)
    path.reverse()
    return [grid_to_world(gx, gy) for gx, gy in path]


def _find_nearest_free(
    grid: list[list[bool]], start_gx: int, start_gy: int, cols: int, rows: int
) -> tuple[int, int]:
    """从 (start_gx, start_gy) 向外 BFS 搜索最近的可通行格子。

    Returns:
        (gx, gy) 或 (-1, -1) 表示找不到
    """
    from collections import deque

    visited: set[tuple[int, int]] = set()
    queue: deque[tuple[int, int]] = deque()
    queue.append((start_gx, start_gy))
    visited.add((start_gx, start_gy))

    while queue:
        gx, gy = queue.popleft()
        if 0 <= gx < cols and 0 <= gy < rows and not grid[gy][gx]:
            return (gx, gy)

        for dx, dy, _ in _NEIGHBORS_8:
            nx, ny = gx + dx, gy + dy
            if (nx, ny) not in visited:
                visited.add((nx, ny))
                queue.append((nx, ny))

    return (-1, -1)


def _default_search_bounds(config: Config) -> tuple[float, float, float, float]:
    """根据机械臂工作空间和棋盘区域生成默认搜索边界。"""
    bx, by = config.base_link_position[0], config.base_link_position[1]
    # 工作空间：以 base_link 为中心，半径 0.25-0.9m
    margin = 0.05
    workspace_radius = 0.85
    return (
        bx - workspace_radius + margin,
        bx + workspace_radius - margin,
        by - workspace_radius + margin,
        by + workspace_radius - margin,
    )


def _mark_obstacle_on_grid(
    grid: list[list[bool]],
    obstacle: Obstacle,
    bounds: tuple[float, float, float, float],
    resolution: float,
    safety_margin: float,
) -> None:
    """在占栅格上标记障碍物。按 shape 分派标记策略。"""
    if obstacle.shape == ObstacleShape.HORIZONTAL_CYLINDER:
        _mark_aabb_on_grid(grid, obstacle, bounds, resolution, safety_margin)
    elif obstacle.shape == ObstacleShape.FLOATING_CUBE:
        _mark_cube_aabb_on_grid(grid, obstacle, bounds, resolution, safety_margin)
    else:
        # VERTICAL_CYLINDER / FLOATING_SPHERE / AABB 及默认：圆形膨胀
        _mark_circle_on_grid(grid, obstacle, bounds, resolution, safety_margin)


def _mark_cube_aabb_on_grid(
    grid: list[list[bool]],
    obstacle: Obstacle,
    bounds: tuple[float, float, float, float],
    resolution: float,
    safety_margin: float,
) -> None:
    """标记轴对齐方形区域（浮空立方体 XY 投影）。

    obstacle.radius = 半边长，无旋转，AABB = [cx±r, cy±r]。
    """
    xmin, xmax, ymin, ymax = bounds
    rows, cols = len(grid), len(grid[0])
    cx, cy = obstacle.center_xyz[0], obstacle.center_xyz[1]
    half = obstacle.radius + safety_margin

    gx_min = max(0, int((cx - half - xmin) / resolution))
    gx_max = min(cols - 1, int((cx + half - xmin) / resolution))
    gy_min = max(0, int((cy - half - ymin) / resolution))
    gy_max = min(rows - 1, int((cy + half - ymin) / resolution))

    for gy in range(gy_min, gy_max + 1):
        for gx in range(gx_min, gx_max + 1):
            grid[gy][gx] = True


def _mark_circle_on_grid(
    grid: list[list[bool]],
    obstacle: Obstacle,
    bounds: tuple[float, float, float, float],
    resolution: float,
    safety_margin: float,
) -> None:
    """标记圆形区域（竖直圆柱投影）。"""
    xmin, xmax, ymin, ymax = bounds
    rows, cols = len(grid), len(grid[0])
    ox, oy = obstacle.center_xyz[0], obstacle.center_xyz[1]
    inflated_radius = obstacle.radius + safety_margin

    # 计算障碍物覆盖的网格范围
    gx_min = max(0, int((ox - inflated_radius - xmin) / resolution))
    gx_max = min(cols - 1, int((ox + inflated_radius - xmin) / resolution))
    gy_min = max(0, int((oy - inflated_radius - ymin) / resolution))
    gy_max = min(rows - 1, int((oy + inflated_radius - ymin) / resolution))

    r2 = inflated_radius * inflated_radius
    for gy in range(gy_min, gy_max + 1):
        for gx in range(gx_min, gx_max + 1):
            wx = xmin + (gx + 0.5) * resolution
            wy = ymin + (gy + 0.5) * resolution
            if (wx - ox) ** 2 + (wy - oy) ** 2 <= r2:
                grid[gy][gx] = True


def _mark_aabb_on_grid(
    grid: list[list[bool]],
    obstacle: Obstacle,
    bounds: tuple[float, float, float, float],
    resolution: float,
    safety_margin: float,
) -> None:
    """标记 AABB 矩形区域（水平圆柱投影）。

    根据 orientation_rpy 判断圆柱主轴方向，生成对应的 AABB。
    """
    import math as _math
    xmin, xmax, ymin, ymax = bounds
    rows, cols = len(grid), len(grid[0])
    cx, cy = obstacle.center_xyz[0], obstacle.center_xyz[1]
    _, pitch, _ = obstacle.orientation_rpy

    if abs(pitch - _math.pi / 2) < 0.01 or abs(pitch + _math.pi / 2) < 0.01:
        # 圆柱沿 X 轴
        half_len_x = obstacle.height / 2.0 + safety_margin
        half_len_y = obstacle.radius + safety_margin
    else:
        # 圆柱沿 Y 轴 / 一般情况：使用包围盒
        half_len_x = obstacle.radius + safety_margin
        half_len_y = obstacle.height / 2.0 + safety_margin

    gx_min = max(0, int((cx - half_len_x - xmin) / resolution))
    gx_max = min(cols - 1, int((cx + half_len_x - xmin) / resolution))
    gy_min = max(0, int((cy - half_len_y - ymin) / resolution))
    gy_max = min(rows - 1, int((cy + half_len_y - ymin) / resolution))

    for gy in range(gy_min, gy_max + 1):
        for gx in range(gx_min, gx_max + 1):
            grid[gy][gx] = True


# ── Theta* 2D path search ──


def _grid_bresenham_clear(
    grid: list[list[bool]],
    gx1: int, gy1: int,
    gx2: int, gy2: int,
    cols: int, rows: int,
) -> bool:
    """Bresenham line-of-sight check on a 2D occupancy grid.

    Returns True if all grid cells along the line from (gx1, gy1)
    to (gx2, gy2) are free (not occupied).
    """
    dx = abs(gx2 - gx1)
    dy = abs(gy2 - gy1)
    x, y = gx1, gy1
    sx = 1 if gx2 > gx1 else -1
    sy = 1 if gy2 > gy1 else -1

    if dx > dy:
        err = dx / 2.0
        while x != gx2:
            if x < 0 or x >= cols or y < 0 or y >= rows:
                return False
            if grid[y][x]:
                return False
            err -= dy
            if err < 0:
                y += sy
                err += dx
            x += sx
    else:
        err = dy / 2.0
        while y != gy2:
            if x < 0 or x >= cols or y < 0 or y >= rows:
                return False
            if grid[y][x]:
                return False
            err -= dx
            if err < 0:
                x += sx
                err += dy
            y += sy

    # Check endpoint
    if 0 <= gx2 < cols and 0 <= gy2 < rows:
        if grid[gy2][gx2]:
            return False
    else:
        return False

    return True


def a_star_theta_2d(
    start_xy: tuple[float, float],
    end_xy: tuple[float, float],
    obstacles: list[Obstacle],
    z_plane: float = 0.18,
    grid_resolution: float = 0.02,
    timeout_ms: float = 100.0,
    bounds: tuple[float, float, float, float] | None = None,
    config: Config = DEFAULT_CONFIG,
) -> PathSearchResult:
    """Theta* 2D path search with Line-of-Sight checks on a z-plane.

    Key difference from A*: when expanding neighbor 's' from current
    node 'curr', checks LoS between parent(curr) and s. If the straight
    line is obstacle-free, the neighbour inherits the parent directly,
    producing paths that are not constrained to 45-degree grid angles.

    Reuses the same grid, heuristic, and reconstruction helpers as A*.

    Args:
        start_xy: start (x, y) in world coordinates
        end_xy: end (x, y) in world coordinates
        obstacles: obstacle list
        z_plane: search plane height (m)
        grid_resolution: grid resolution (m)
        timeout_ms: search timeout (ms)
        bounds: (xmin, xmax, ymin, ymax); defaults to workspace
        config: configuration object

    Returns:
        PathSearchResult(success, path_xy, search_time_ms, nodes_explored)
    """
    t_start = time.perf_counter()

    if bounds is None:
        bounds = _default_search_bounds(config)

    xmin, xmax, ymin, ymax = bounds
    cols = max(1, int(math.ceil((xmax - xmin) / grid_resolution)))
    rows = max(1, int(math.ceil((ymax - ymin) / grid_resolution)))

    grid = build_2d_occupancy_grid(
        obstacles, z_plane, grid_resolution, bounds,
        safety_margin=config.safety_margin,
    )

    # World <-> grid index conversion
    def world_to_grid(x: float, y: float) -> tuple[int, int]:
        gx = int((x - xmin) / grid_resolution)
        gy = int((y - ymin) / grid_resolution)
        return (max(0, min(cols - 1, gx)), max(0, min(rows - 1, gy)))

    def grid_to_world(gx: int, gy: int) -> tuple[float, float]:
        return (xmin + (gx + 0.5) * grid_resolution,
                ymin + (gy + 0.5) * grid_resolution)

    start_gx, start_gy = world_to_grid(*start_xy)
    end_gx, end_gy = world_to_grid(*end_xy)

    # Push start/goal out of obstacles if needed
    if grid[start_gy][start_gx]:
        start_gx, start_gy = _find_nearest_free(grid, start_gx, start_gy, cols, rows)
        if start_gx < 0:
            return PathSearchResult(
                success=False,
                search_time_ms=(time.perf_counter() - t_start) * 1000,
            )

    if grid[end_gy][end_gx]:
        end_gx, end_gy = _find_nearest_free(grid, end_gx, end_gy, cols, rows)
        if end_gx < 0:
            return PathSearchResult(
                success=False,
                search_time_ms=(time.perf_counter() - t_start) * 1000,
            )

    if (start_gx, start_gy) == (end_gx, end_gy):
        return PathSearchResult(
            success=True,
            path_xy=[grid_to_world(start_gx, start_gy)],
            search_time_ms=(time.perf_counter() - t_start) * 1000,
            nodes_explored=0,
        )

    # ── Theta* search core ──
    start_key = (start_gx, start_gy)
    end_key = (end_gx, end_gy)

    open_set: list[tuple[float, int, tuple[int, int]]] = []
    tie_breaker = 0
    h_start = _heuristic(start_gx, start_gy, end_gx, end_gy)
    heapq.heappush(open_set, (h_start, tie_breaker, start_key))
    tie_breaker += 1

    g_score: dict[tuple[int, int], float] = {start_key: 0.0}
    came_from: dict[tuple[int, int], tuple[int, int]] = {}
    # came_from[start_key] is left unset; LoS checks treat missing parent
    # as standard A* fallback.
    closed_set: set[tuple[int, int]] = set()
    nodes_explored = 0
    timeout_sec = timeout_ms / 1000.0

    while open_set:
        if time.perf_counter() - t_start > timeout_sec:
            return PathSearchResult(
                success=False,
                search_time_ms=(time.perf_counter() - t_start) * 1000,
                nodes_explored=nodes_explored,
            )

        _, _, current = heapq.heappop(open_set)
        if current in closed_set:
            continue

        nodes_explored += 1

        if current == end_key:
            path_xy = _reconstruct_path(came_from, current, grid_to_world)
            return PathSearchResult(
                success=True,
                path_xy=path_xy,
                search_time_ms=(time.perf_counter() - t_start) * 1000,
                nodes_explored=nodes_explored,
            )

        closed_set.add(current)
        cx, cy = current

        # Parent of current node (for LoS checks)
        parent_curr = came_from.get(current)

        for dx, dy, move_cost in _NEIGHBORS_8:
            nx, ny = cx + dx, cy + dy
            if nx < 0 or nx >= cols or ny < 0 or ny >= rows:
                continue
            if grid[ny][nx]:
                continue
            neighbor = (nx, ny)
            if neighbor in closed_set:
                continue

            # ── Theta* LoS check ──
            if parent_curr is not None and _grid_bresenham_clear(
                grid, parent_curr[0], parent_curr[1], nx, ny, cols, rows,
            ):
                # Path 2 via parent (direct LoS shortcut)
                px, py = parent_curr
                dist_grid = math.hypot(nx - px, ny - py)
                tentative_g = g_score[parent_curr] + dist_grid * grid_resolution
                if tentative_g < g_score.get(neighbor, float("inf")):
                    g_score[neighbor] = tentative_g
                    came_from[neighbor] = parent_curr
                    f = tentative_g + _heuristic(nx, ny, end_gx, end_gy)
                    heapq.heappush(open_set, (f, tie_breaker, neighbor))
                    tie_breaker += 1
            else:
                # Standard A* update from current
                tentative_g = g_score[current] + move_cost * grid_resolution
                if tentative_g < g_score.get(neighbor, float("inf")):
                    g_score[neighbor] = tentative_g
                    came_from[neighbor] = current
                    f = tentative_g + _heuristic(nx, ny, end_gx, end_gy)
                    heapq.heappush(open_set, (f, tie_breaker, neighbor))
                    tie_breaker += 1

    return PathSearchResult(
        success=False,
        search_time_ms=(time.perf_counter() - t_start) * 1000,
        nodes_explored=nodes_explored,
    )
