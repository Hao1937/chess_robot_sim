from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class Config:
    board_cols: int = 9
    board_rows: int = 10
    board_origin: tuple[float, float, float] = (0.0, 0.0, 0.0)
    cell_size: float = 0.06
    z_board: float = 0.0
    z_grasp: float = 0.055
    z_safe: float = 0.18
    piece_radius: float = 0.0225
    piece_height: float = 0.018
    end_effector_radius: float = 0.012
    safety_margin: float = 0.015
    home_pose: tuple[float, ...] = (0.0, -0.8, 1.2, -0.4, 0.0, 0.0)
    fast_speed_scale: float = 1.0
    safe_speed_scale: float = 0.35
    base_link_position: tuple[float, float, float] = (0.32, -0.25, 0.0)
    base_link_orientation_rpy: tuple[float, float, float]=(0, 0, 0)

    @property
    def inflated_piece_radius(self) -> float:
        return self.piece_radius + self.end_effector_radius + self.safety_margin


DEFAULT_CONFIG = Config()
