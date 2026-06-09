from __future__ import annotations

from src.common.types import MoveCommand


def parse_command(command_text: str) -> MoveCommand:
    """Parse CLI text into a MoveCommand.

    Supported mock commands:
    - "A1 B1" for a move.
    - "reset" for reset.
    - "hand_on" / "hand_off" for human safety zone simulation.
    - "obstacle_mode 1" / "obstacle_mode 2" / "obstacle_mode 3" for preset column obstacles.
    """
    text = command_text.strip().upper()
    if not text:
        raise ValueError("empty command")
    if text.lower() in {"reset", "hand_on", "hand_off"}:
        return MoveCommand(command_type=text.lower())
    parts = text.split()
    if len(parts) == 2 and parts[0].lower() == "obstacle_mode":
        if parts[1] not in {"1", "2", "3"}:
            raise ValueError("obstacle_mode must be 1, 2, or 3")
        return MoveCommand(command_type="obstacle_mode", mode=f"mode_{parts[1]}")
    if len(parts) != 2:
        raise ValueError("move command must look like: A1 B1")
    return MoveCommand(command_type="move", from_cell=parts[0], to_cell=parts[1])
