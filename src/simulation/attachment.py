from __future__ import annotations

from src.common.types import OperationResult
from src.simulation._runtime import RUNTIME, ensure_client, p


def attach_piece(piece_id: str, end_effector_id: int) -> OperationResult:
    """Attach a piece to the end effector using a virtual constraint."""
    if p is None:
        return OperationResult(True, f"mock attached {piece_id} to end effector {end_effector_id}")

    client_id = ensure_client()
    body_id = RUNTIME.piece_body_ids.get(piece_id)
    if client_id is None or RUNTIME.robot_id is None:
        return OperationResult(True, f"mock attached {piece_id}; robot is not loaded")
    if body_id is None:
        return OperationResult(False, f"unknown piece id: {piece_id}")

    old_constraint = RUNTIME.attachment_constraints.pop(piece_id, None)
    if old_constraint is not None:
        p.removeConstraint(old_constraint, physicsClientId=client_id)

    ee_state = p.getLinkState(RUNTIME.robot_id, end_effector_id, physicsClientId=client_id)
    if ee_state is not None:
        p.resetBasePositionAndOrientation(
            body_id,
            ee_state[0],
            ee_state[1],
            physicsClientId=client_id,
        )

    constraint_id = p.createConstraint(
        parentBodyUniqueId=RUNTIME.robot_id,
        parentLinkIndex=end_effector_id,
        childBodyUniqueId=body_id,
        childLinkIndex=-1,
        jointType=p.JOINT_FIXED,
        jointAxis=(0.0, 0.0, 0.0),
        parentFramePosition=(0.0, 0.0, 0.0),
        childFramePosition=(0.0, 0.0, 0.0),
        physicsClientId=client_id,
    )
    RUNTIME.attachment_constraints[piece_id] = constraint_id
    return OperationResult(True, f"attached {piece_id} to end effector {end_effector_id}")


def detach_piece(piece_id: str) -> OperationResult:
    """Detach a piece from the end effector."""
    if p is None:
        return OperationResult(True, f"mock detached {piece_id}")

    client_id = ensure_client()
    constraint_id = RUNTIME.attachment_constraints.pop(piece_id, None)
    if client_id is None:
        return OperationResult(True, f"mock detached {piece_id}; no active client")
    if constraint_id is None:
        return OperationResult(True, f"{piece_id} was not attached")

    p.removeConstraint(constraint_id, physicsClientId=client_id)
    return OperationResult(True, f"detached {piece_id}")
