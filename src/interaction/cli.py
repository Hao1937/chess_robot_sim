from __future__ import annotations

import re

from src.common.types import MoveCommand


_CELL_PATTERN = re.compile(r"^[A-I](?:[1-9]|10)$")


def parse_command(command_text: str) -> MoveCommand:
    """Parse CLI text into a MoveCommand.

    Supported mock commands:
    - "A1 B1" for a move.
    - "reset" for reset.
    - "hand_on" / "hand_off" for human safety zone simulation.
    - "obstacle_mode 1" / "obstacle_mode 2" / "obstacle_mode 3" for preset column obstacles.
    """
    text = command_text.strip()
    if not text:
        raise ValueError("empty command")

    parts = text.split()
    lowered = [part.lower() for part in parts]

    if len(parts) == 1 and lowered[0] in {"reset", "hand_on", "hand_off"}:
        return MoveCommand(command_type=lowered[0])

    if len(parts) == 2 and lowered[0] == "obstacle_mode":
        if parts[1] not in {"1", "2", "3"}:
            raise ValueError("obstacle_mode must be 1, 2, or 3")
        return MoveCommand(command_type="obstacle_mode", mode=f"mode_{parts[1]}")

    if len(parts) != 2:
        raise ValueError("move command must look like: A1 B1")

    from_cell = parts[0].upper()
    to_cell = parts[1].upper()
    if not _is_valid_cell(from_cell) or not _is_valid_cell(to_cell):
        raise ValueError("move cells must be board coordinates like A1 or I10")

    return MoveCommand(command_type="move", from_cell=from_cell, to_cell=to_cell)


def _is_valid_cell(cell: str) -> bool:
    return bool(_CELL_PATTERN.fullmatch(cell))
