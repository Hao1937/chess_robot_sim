from __future__ import annotations

from src.common.config import DEFAULT_CONFIG, Config
from src.common.types import Obstacle, SafetyDecision
from src.planning.chessboard_mapping import cell_to_world
from src.planning.ik_solver import is_reachable


def build_obstacle_map(
    piece_cells: list[str],
    extra_obstacles: list[Obstacle],
    human_hand_present: bool = False,
    config: Config = DEFAULT_CONFIG,
) -> list[Obstacle]:
    """Build inflated obstacle list from pieces, scene obstacles, and hand zone."""
    obstacles = [
        Obstacle(
            obstacle_id=f"piece_{cell}",
            center_xyz=cell_to_world(cell, config),
            radius=config.inflated_piece_radius,
            height=config.piece_height,
            dynamic=False,
        )
        for cell in piece_cells
        if not cell.startswith("CAPTURED_")
    ]
    obstacles.extend(extra_obstacles)
    if human_hand_present:
        obstacles.append(
            Obstacle(
                obstacle_id="human_hand_zone",
                center_xyz=(config.board_origin[0] + 4 * config.cell_size, config.board_origin[1] + 4 * config.cell_size, config.z_board),
                radius=0.08,
                height=0.12,
                dynamic=True,
            )
        )
    return obstacles


def assess_obstacle_intervention(
    target_xyz: tuple[float, float, float],
    obstacles: list[Obstacle],
    config: Config = DEFAULT_CONFIG,
) -> SafetyDecision:
    """Decide whether a dynamic obstacle allows replanning, safe motion, or pause."""
    if not is_reachable(target_xyz, config):
        return SafetyDecision(status="pause", reason="target unreachable after obstacle update")

    for obstacle in obstacles:
        if not obstacle.dynamic:
            continue
        ox, oy, _ = obstacle.center_xyz
        x, y, _ = target_xyz
        distance_xy = ((x - ox) ** 2 + (y - oy) ** 2) ** 0.5
        if distance_xy <= obstacle.radius:
            return SafetyDecision(status="pause", reason=f"target blocked by {obstacle.obstacle_id}")
        if distance_xy <= obstacle.radius + config.inflated_piece_radius:
            return SafetyDecision(status="safe", reason=f"target near {obstacle.obstacle_id}; use safe mode")

    return SafetyDecision(status="continue", reason="target reachable and clear")
