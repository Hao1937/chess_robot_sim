from __future__ import annotations

from src.common.config import DEFAULT_CONFIG, Config


def cell_to_world(cell: str, config: Config = DEFAULT_CONFIG) -> tuple[float, float, float]:
    """Map a board cell like A1 to a world-frame target point."""
    if cell.startswith("CAPTURED_"):
        return _captured_cell_to_world(cell, config)
    col = ord(cell[0].upper()) - ord("A")
    row = int(cell[1:]) - 1
    x0, y0, _ = config.board_origin
    return (x0 + col * config.cell_size, y0 + row * config.cell_size, config.z_board)


def cell_above_world(cell: str, config: Config = DEFAULT_CONFIG) -> tuple[float, float, float]:
    x, y, _ = cell_to_world(cell, config)
    return (x, y, config.z_safe)


def _captured_cell_to_world(cell: str, config: Config) -> tuple[float, float, float]:
    suffix = int(cell.rsplit("_", 1)[-1])
    x0, y0, _ = config.board_origin
    return (x0 + (config.board_cols + 1) * config.cell_size, y0 + suffix * config.cell_size, config.z_board)
