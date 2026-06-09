from __future__ import annotations

from src.common.config import DEFAULT_CONFIG, Config
from src.common.types import Obstacle, SceneHandle


def build_scene(config: Config = DEFAULT_CONFIG, obstacle_mode: str = "mode_1") -> SceneHandle:
    """Build the table, chessboard, pieces, captured area, and obstacles."""
    obstacles = build_obstacle_preset(obstacle_mode, config)
    return SceneHandle(
        board_id=100,
        piece_ids={"A1": 201, "B1": 202, "C1": 203},
        obstacles=obstacles,
    )


def build_obstacle_preset(obstacle_mode: str, config: Config = DEFAULT_CONFIG) -> list[Obstacle]:
    """Return 2-3 preset vertical cylinder obstacles for avoidance demos."""
    presets = {
        "mode_1": [(2, 1), (4, 1)],
        "mode_2": [(3, 2), (4, 4), (6, 3)],
        "mode_3": [(1, 3), (5, 5), (7, 2)],
        "none": [],
    }
    cells = presets.get(obstacle_mode)
    if cells is None:
        raise ValueError(f"unknown obstacle_mode: {obstacle_mode}")
    return [
        Obstacle(
            obstacle_id=f"preset_column_{index + 1}",
            center_xyz=(
                config.board_origin[0] + col * config.cell_size,
                config.board_origin[1] + row * config.cell_size,
                config.z_board,
            ),
            radius=config.inflated_piece_radius,
            height=0.08,
            dynamic=False,
        )
        for index, (col, row) in enumerate(cells)
    ]
