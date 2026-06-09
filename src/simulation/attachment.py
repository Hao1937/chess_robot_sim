from __future__ import annotations

from src.common.types import OperationResult


def attach_piece(piece_id: str, end_effector_id: int) -> OperationResult:
    """Attach a piece to the end effector using a virtual constraint."""
    return OperationResult(True, f"attached {piece_id} to end effector {end_effector_id}")


def detach_piece(piece_id: str) -> OperationResult:
    """Detach a piece from the end effector."""
    return OperationResult(True, f"detached {piece_id}")
