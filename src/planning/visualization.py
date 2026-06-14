"""路径规划可视化 —— PyBullet GUI 调试线条。

在 PyBullet 场景中用彩色线条展示规划路径 vs 直接路径。
"""

from __future__ import annotations

from src.simulation._runtime import RUNTIME, p


def draw_path_debug(
    path_xyz: list[tuple[float, float, float]],
    color: tuple[float, float, float] = (1.0, 0.0, 0.0),
    *,
    line_width: float = 2.5,
    life_time: float = 8.0,
) -> None:
    """在 PyBullet 中用 addUserDebugLine 绘制路径连线。

    Args:
        path_xyz: 路径点列表 (x, y, z)
        color: RGB 颜色，默认红色
        line_width: 线宽 (px)
        life_time: 显示持续时间 (s)，超时自动消失
    """
    if p is None:
        return
    client_id = RUNTIME.client_id
    if client_id is None or not p.isConnected(client_id):
        return

    for i in range(len(path_xyz) - 1):
        item_id = p.addUserDebugLine(
            path_xyz[i],
            path_xyz[i + 1],
            lineColorRGB=color,
            lineWidth=line_width,
            lifeTime=life_time,
            physicsClientId=client_id,
        )
        RUNTIME.debug_item_ids.append(item_id)


def draw_direct_line(
    start: tuple[float, float, float],
    end: tuple[float, float, float],
    color: tuple[float, float, float] = (0.0, 1.0, 0.0),
    *,
    line_width: float = 1.5,
    life_time: float = 8.0,
) -> None:
    """绘制起点→终点的直线（绿色虚线表示无障碍的最短路径）。

    Args:
        start: 起点 (x, y, z)
        end: 终点 (x, y, z)
        color: RGB 颜色，默认绿色
        line_width: 线宽 (px)
        life_time: 显示持续时间 (s)
    """
    if p is None:
        return
    client_id = RUNTIME.client_id
    if client_id is None or not p.isConnected(client_id):
        return

    item_id = p.addUserDebugLine(
        start,
        end,
        lineColorRGB=color,
        lineWidth=line_width,
        lifeTime=life_time,
        physicsClientId=client_id,
    )
    RUNTIME.debug_item_ids.append(item_id)
