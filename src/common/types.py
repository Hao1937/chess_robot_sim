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


@dataclass(frozen=True)
class Obstacle:
    obstacle_id: str
    center_xyz: tuple[float, float, float]
    radius: float
    height: float
    dynamic: bool = False


@dataclass(frozen=True)
class SafetyDecision:
    status: str
    reason: str = ""


@dataclass(frozen=True)
class JointTrajectory:
    joint_waypoints: list[tuple[float, ...]]
    speed_profile: list[str]


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
