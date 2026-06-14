from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum


class PieceColor(str, Enum):
    RED = "red"
    BLACK = "black"


class PieceType(str, Enum):
    ROOK = "rook"
    HORSE = "horse"
    CANNON = "cannon"
    GENERAL = "general"
    ADVISOR = "advisor"
    ELEPHANT = "elephant"
    SOLDIER = "soldier"


class ObstacleShape(str, Enum):
    """障碍物形状类型，用于碰撞检测和占栅格生成的分派。"""
    VERTICAL_CYLINDER = "vertical_cylinder"    # 竖直圆柱（棋子、预设柱）
    HORIZONTAL_CYLINDER = "horizontal_cylinder" # 水平横躺圆柱（人手安全区）
    AABB = "aabb"                               # 轴对齐包围盒（预留扩展）


@dataclass(frozen=True)
class Piece:
    piece_id: str
    kind: PieceType
    color: PieceColor
    cell: str


@dataclass
class BoardState:
    pieces: dict[str, Piece] = field(default_factory=dict)
    captured_counts: dict[PieceColor, int] = field(default_factory=lambda: {
        PieceColor.RED: 0,
        PieceColor.BLACK: 0,
    })


@dataclass(frozen=True)
class MoveCommand:
    command_type: str
    from_cell: str = ""
    to_cell: str = ""
    mode: str = ""


@dataclass(frozen=True)
class ValidationResult:
    is_legal: bool
    reason: str = ""


@dataclass(frozen=True)
class LogicalAction:
    action_type: str
    cell: str
    piece_id: str = ""


@dataclass(frozen=True)
class MotionPrimitive:
    primitive_type: str
    cell: str
    target_xyz: tuple[float, float, float]
    speed_mode: str = "safe"
    piece_id: str = ""


@dataclass(frozen=True)
class Obstacle:
    obstacle_id: str
    center_xyz: tuple[float, float, float]
    radius: float
    height: float
    dynamic: bool = False
    shape: ObstacleShape = ObstacleShape.VERTICAL_CYLINDER
    orientation_rpy: tuple[float, float, float] = (0.0, 0.0, 0.0)


@dataclass(frozen=True)
class PathSearchResult:
    """A* / RRT 路径搜索结果"""
    success: bool
    path_xy: list[tuple[float, float]] = field(default_factory=list)
    search_time_ms: float = 0.0
    nodes_explored: int = 0


@dataclass(frozen=True)
class CollisionCheckResult:
    """路径段碰撞检测结果"""
    collision_free: bool
    min_clearance: float = float("inf")
    collision_point: tuple[float, float, float] | None = None


@dataclass(frozen=True)
class SmoothingResult:
    """轨迹平滑结果"""
    original_count: int
    smoothed_count: int
    max_jerk_reduction: float = 0.0


@dataclass(frozen=True)
class SafetyDecision:
    status: str
    reason: str = ""


@dataclass(frozen=True)
class PrimitivePlanningContext:
    primitive: MotionPrimitive
    obstacles: list[Obstacle]
    safety_decision: SafetyDecision


@dataclass(frozen=True)
class JointTrajectory:
    joint_waypoints: list[tuple[float, ...]]
    speed_profile: list[str]
    primitive_ranges: list[tuple[int, int]] | None = None
    """每个 MotionPrimitive 在 joint_waypoints 中的 [start, end) 索引。
    由 plan_trajectory() 填充；main.py 用于按 action 分段执行。"""


@dataclass(frozen=True)
class RobotHandle:
    robot_id: int
    end_effector_id: int
    joint_indices: tuple[int, ...]


@dataclass(frozen=True)
class SceneHandle:
    board_id: int
    piece_ids: dict[str, int]
    obstacles: list[Obstacle]


@dataclass(frozen=True)
class OperationResult:
    success: bool
    message: str = ""


@dataclass(frozen=True)
class ExecutionResult:
    success: bool
    desired_joint_angles: list[tuple[float, ...]]
    actual_joint_angles: list[tuple[float, ...]]
    joint_errors: list[float]
    end_effector_errors: list[float]
    obstacle_clearances: list[float]
    execution_time: float
