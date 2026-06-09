from __future__ import annotations

from src.common.types import MoveCommand


def parse_command(command_text: str) -> MoveCommand:
    """Parse CLI text into a MoveCommand.

    Supported mock commands:
    - "A1 B1" for a move.
    - "reset" for reset.
    - "hand_on" / "hand_off" for human safety zone simulation.
    """
    text = command_text.strip().upper()
    if not text:
        raise ValueError("empty command")
    if text.lower() in {"reset", "hand_on", "hand_off"}:
        return MoveCommand(command_type=text.lower())
    parts = text.split()
    if len(parts) != 2:
        raise ValueError("move command must look like: A1 B1")
    return MoveCommand(command_type="move", from_cell=parts[0], to_cell=parts[1])
