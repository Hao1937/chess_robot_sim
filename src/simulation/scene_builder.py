from __future__ import annotations

from src.common.config import DEFAULT_CONFIG, Config
from src.common.types import Obstacle, SceneHandle


def build_scene(config: Config = DEFAULT_CONFIG) -> SceneHandle:
    """Build the table, chessboard, pieces, captured area, and obstacles."""
    obstacle = Obstacle(
        obstacle_id="static_column_1",
        center_xyz=(config.board_origin[0] + 2 * config.cell_size, config.board_origin[1], config.z_board),
        radius=config.inflated_piece_radius,
        height=0.08,
        dynamic=False,
    )
    return SceneHandle(
        board_id=100,
        piece_ids={"A1": 201, "B1": 202, "C1": 203},
        obstacles=[obstacle],
    )
