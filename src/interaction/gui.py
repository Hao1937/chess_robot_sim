from __future__ import annotations

from typing import Callable

from src.common.types import BoardState, MoveCommand
from src.interaction.chinese_notation import parse_chinese_move
from src.interaction.cli import parse_command


InputFunc = Callable[[str], str]


def make_gui_command(from_cell: str, to_cell: str) -> MoveCommand:
    """Adapter for one GUI click selection."""
    return MoveCommand(command_type="move", from_cell=from_cell.upper(), to_cell=to_cell.upper())


def make_safety_command(hand_present: bool) -> MoveCommand:
    """Return a command representing human hand entering/leaving workspace."""
    return MoveCommand(command_type="hand_on" if hand_present else "hand_off")


def poll_gui_command(board: BoardState, input_func: InputFunc = input, prompt: str = "move> ") -> MoveCommand | None:
    """Poll one GUI/input event and convert it to a MoveCommand.

    The current implementation uses a text input function so the main session can
    run continuously before the real GUI is finished. A later GUI can keep this
    contract and return one command per user event.
    """
    text = input_func(prompt).strip()
    if not text:
        return None
    if text.lower() in {"quit", "exit"}:
        return MoveCommand(command_type="quit")

    try:
        return parse_command(text)
    except ValueError:
        return parse_chinese_move(text, board)
