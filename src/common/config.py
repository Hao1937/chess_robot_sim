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
    human_hand_zone_col: int = 4
    human_hand_zone_row: int = 7
    human_hand_zone_radius: float = 0.025
    human_hand_zone_length_cells: float = 4.0

    # ── Path Planning 参数 ──
    path_grid_resolution: float = 0.02        # A* 网格分辨率 (m)
    path_collision_check_step: float = 0.005  # 碰撞检测步长 (m)
    path_search_timeout_ms: float = 100.0     # 搜索超时 (ms)
    path_smoothing_angle_threshold: float = 0.15  # shortcut 角度阈值 (rad)

    # ── Waypoint Interpolation ──
    waypoint_interpolation_step: float = 0.03  # 插值步长 (m) in Cartesian
    waypoint_vertical_step: float = 0.01       # 垂直方向插值步长 (m)

    @property
    def inflated_piece_radius(self) -> float:
        return self.piece_radius + self.end_effector_radius + self.safety_margin

    @property
    def human_hand_zone_center(self) -> tuple[float, float, float]:
        return (
            self.board_origin[0] + self.human_hand_zone_col * self.cell_size,
            self.board_origin[1] + self.human_hand_zone_row * self.cell_size,
            self.z_safe,
        )

    @property
    def human_hand_zone_length(self) -> float:
        return self.human_hand_zone_length_cells * self.cell_size

    @property
    def human_hand_planning_radius(self) -> float:
        return self.human_hand_zone_length / 2.0


DEFAULT_CONFIG = Config()
