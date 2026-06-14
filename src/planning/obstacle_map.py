from __future__ import annotations

from src.common.config import DEFAULT_CONFIG, Config
from src.common.types import BoardState, LogicalAction, MotionPrimitive, Obstacle, PrimitivePlanningContext, SafetyDecision
from src.planning.chessboard_mapping import cell_to_world
from src.planning.ik_solver import is_reachable


def build_obstacle_map(
    piece_cells: list[str],
    extra_obstacles: list[Obstacle],
    human_hand_present: bool = False,
    config: Config = DEFAULT_CONFIG,
    excluded_cells: set[str] | None = None,
) -> list[Obstacle]:
    """Build inflated obstacle list from pieces, scene obstacles, and hand zone."""
    excluded_cells = excluded_cells or set()
    obstacles = [
        Obstacle(
            obstacle_id=f"piece_{cell}",
            center_xyz=cell_to_world(cell, config),
            radius=config.inflated_piece_radius,
            height=config.piece_height,
            dynamic=False,
        )
        for cell in piece_cells
        if not cell.startswith("CAPTURED_") and cell not in excluded_cells
    ]
    obstacles.extend(extra_obstacles)
    if human_hand_present:
        obstacles.append(
            Obstacle(
                obstacle_id="human_hand_zone",
                center_xyz=config.human_hand_zone_center,
                radius=config.human_hand_planning_radius,
                height=config.human_hand_zone_radius * 2.0,
                dynamic=True,
            )
        )
    return obstacles


def build_primitive_obstacle_contexts(
    actions: list[LogicalAction],
    primitives: list[MotionPrimitive],
    board: BoardState,
    extra_obstacles: list[Obstacle],
    human_hand_present: bool = False,
    config: Config = DEFAULT_CONFIG,
) -> list[PrimitivePlanningContext]:
    """Build obstacle and safety context for each primitive.

    The context simulates the board occupancy across a pick/place sequence:
    a piece stops being an environment obstacle once it is lifted, and a placed
    piece becomes an obstacle again during the retreat phase.
    """
    _ = actions  # The primitive stream is derived from these actions; kept for interface clarity.
    occupied_cells = {
        cell
        for cell in board.pieces
        if not cell.startswith("CAPTURED_")
    }

    contexts: list[PrimitivePlanningContext] = []
    for primitive in primitives:
        if primitive.primitive_type == "lift":
            occupied_cells.discard(primitive.cell)
        elif primitive.primitive_type == "retreat" and primitive.cell:
            occupied_cells.add(primitive.cell)

        obstacles = build_obstacle_map(
            piece_cells=sorted(occupied_cells),
            extra_obstacles=extra_obstacles,
            human_hand_present=human_hand_present,
            config=config,
        )
        contexts.append(
            PrimitivePlanningContext(
                primitive=primitive,
                obstacles=obstacles,
                safety_decision=assess_obstacle_intervention(primitive.target_xyz, obstacles, config),
            )
        )
    return contexts


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
