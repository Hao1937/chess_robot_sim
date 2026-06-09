from __future__ import annotations

from src.common.types import MoveCommand


def make_gui_command(from_cell: str, to_cell: str) -> MoveCommand:
    """Placeholder adapter for a future GUI click selection."""
    return MoveCommand(command_type="move", from_cell=from_cell.upper(), to_cell=to_cell.upper())


def make_safety_command(hand_present: bool) -> MoveCommand:
    """Return a command representing human hand entering/leaving workspace."""
    return MoveCommand(command_type="hand_on" if hand_present else "hand_off")
