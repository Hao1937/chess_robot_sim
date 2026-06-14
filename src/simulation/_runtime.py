from __future__ import annotations

from dataclasses import dataclass, field
import os
from pathlib import Path

try:
    import pybullet as p
except ModuleNotFoundError:  # pragma: no cover - exercised when dependency is absent
    p = None

try:
    import pybullet_data
except ModuleNotFoundError:  # pragma: no cover - exercised when dependency is absent
    pybullet_data = None


@dataclass
class SimulationRuntime:
    client_id: int | None = None
    robot_id: int | None = None
    end_effector_id: int | None = None
    joint_indices: tuple[int, ...] = ()
    scene_body_ids: list[int] = field(default_factory=list)
    debug_item_ids: list[int] = field(default_factory=list)
    piece_body_ids: dict[str, int] = field(default_factory=dict)
    piece_cells: dict[str, str] = field(default_factory=dict)
    piece_ids_by_cell: dict[str, str] = field(default_factory=dict)
    attachment_constraints: dict[str, int] = field(default_factory=dict)
    manually_attached_pieces: dict[str, int] = field(default_factory=dict)
    # piece_id → end_effector_id 的手动吸附映射（非约束方案）
    human_zone_body_id: int | None = None


RUNTIME = SimulationRuntime()


def project_root() -> Path:
    return Path(__file__).resolve().parents[2]


def pybullet_available() -> bool:
    return p is not None


def ensure_client() -> int | None:
    """Return a PyBullet client id, creating a client when needed."""
    if p is None:
        return None
    if RUNTIME.client_id is not None and p.isConnected(RUNTIME.client_id):
        return RUNTIME.client_id
    connection_mode = p.GUI if os.environ.get("CHESS_ROBOT_PYBULLET_GUI") == "1" else p.DIRECT
    connect_kwargs = {}
    if connection_mode == p.GUI:
        width = os.environ.get("CHESS_ROBOT_GUI_WIDTH", "1600")
        height = os.environ.get("CHESS_ROBOT_GUI_HEIGHT", "1000")
        connect_kwargs["options"] = f"--width={width} --height={height}"
    RUNTIME.client_id = p.connect(connection_mode, **connect_kwargs)
    if pybullet_data is not None:
        p.setAdditionalSearchPath(pybullet_data.getDataPath(), physicsClientId=RUNTIME.client_id)
    p.setGravity(0.0, 0.0, -9.81, physicsClientId=RUNTIME.client_id)
    return RUNTIME.client_id


def clear_scene_bodies() -> None:
    """Remove previously created scene bodies while leaving the robot loaded."""
    if p is None or RUNTIME.client_id is None or not p.isConnected(RUNTIME.client_id):
        return
    for constraint_id in RUNTIME.attachment_constraints.values():
        try:
            p.removeConstraint(constraint_id, physicsClientId=RUNTIME.client_id)
        except Exception:
            pass
    for body_id in RUNTIME.scene_body_ids:
        try:
            p.removeBody(body_id, physicsClientId=RUNTIME.client_id)
        except Exception:
            pass
    for debug_id in RUNTIME.debug_item_ids:
        try:
            p.removeUserDebugItem(debug_id, physicsClientId=RUNTIME.client_id)
        except Exception:
            pass
    RUNTIME.scene_body_ids.clear()
    RUNTIME.debug_item_ids.clear()
    RUNTIME.piece_body_ids.clear()
    RUNTIME.piece_cells.clear()
    RUNTIME.piece_ids_by_cell.clear()
    RUNTIME.attachment_constraints.clear()
    RUNTIME.manually_attached_pieces.clear()
    RUNTIME.human_zone_body_id = None
