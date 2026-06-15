from __future__ import annotations

import sys
from typing import TYPE_CHECKING, Callable

if TYPE_CHECKING:
    from src.interaction.board_gui import BoardGUI

from src.common.types import BoardState, MoveCommand
from src.interaction.cli import parse_command


InputFunc = Callable[[str], str]

# Module-level reference to the active board GUI (set by create_board_gui)
_active_board_gui: BoardGUI | None = None


def create_board_gui(board: BoardState) -> BoardGUI:
    """Create and activate a clickable Chinese chess board GUI window.

    After calling this, poll_gui_command() will also consume click events from
    the board window (in addition to text input).  The returned BoardGUI instance
    can be used directly for finer control.
    """
    global _active_board_gui
    from src.interaction.board_gui import BoardGUI

    _active_board_gui = BoardGUI(board)
    return _active_board_gui


def get_active_board_gui() -> BoardGUI | None:
    """Return the currently active BoardGUI instance, or None."""
    return _active_board_gui


def make_gui_command(from_cell: str, to_cell: str) -> MoveCommand:
    """Adapter for one GUI click selection."""
    return MoveCommand(command_type="move", from_cell=from_cell.upper(), to_cell=to_cell.upper())


def make_safety_command(hand_present: bool) -> MoveCommand:
    """Return a command representing human hand entering/leaving workspace."""
    return MoveCommand(command_type="hand_on" if hand_present else "hand_off")


def poll_gui_command(board: BoardState, input_func: InputFunc = input, prompt: str = "move> ") -> MoveCommand | None:
    """Poll one GUI/input event and convert it to a MoveCommand.

    When a BoardGUI is active, click events are the PRIMARY input — the function
    pumps the GUI event loop and returns immediately, never blocking on stdin.
    Text input is only used as fallback when no BoardGUI is registered.
    """
    if _active_board_gui is not None:
        # Board GUI active — pump its event loop, check for click commands.
        # Also check for window close.
        if not _active_board_gui.is_open:
            _active_board_gui.close()
            return MoveCommand(command_type="quit")

        cmd = _active_board_gui.get_next_command()
        if cmd is not None:
            return cmd

        # No click command ready yet — return None so the main loop keeps
        # polling.  This keeps the matplotlib event loop alive and the
        # window responsive.  Text input is still available via the
        # terminal (type commands while the board window has focus).
        import sys
        import select

        # Non-blocking check for stdin (terminal text commands)
        if sys.stdin.isatty():
            try:
                import msvcrt
                if msvcrt.kbhit():
                    # 用 getwch() 读宽字符（Windows 控制台内部 UTF-16），
                    # 避免 getch() 字节流在中英文编码间出错（GBK vs UTF-8）
                    chars: list[str] = []
                    while msvcrt.kbhit():
                        ch = msvcrt.getwch()  # 宽字符，不回显（IME 自行处理）
                        if ch in ('\r', '\n'):
                            break
                        chars.append(ch)
                    line = ''.join(chars).strip()
                    if not line:
                        return None
                    if line.lower() in {"quit", "exit"}:
                        return MoveCommand(command_type="quit")
                    try:
                        return parse_command(line, board=board)
                    except ValueError as exc:
                        print(f"error: {exc}", file=sys.stderr)
                        return None
            except ImportError:
                # Unix fallback
                rlist, _, _ = select.select([sys.stdin], [], [], 0)
                if rlist:
                    text = input_func(prompt).strip()
                    if not text:
                        return None
                    if text.lower() in {"quit", "exit"}:
                        return MoveCommand(command_type="quit")
                    try:
                        return parse_command(text, board=board)
                    except ValueError as exc:
                        print(f"error: {exc}", file=sys.stderr)
                        return None

        return None

    # ── No BoardGUI active — fallback to blocking text input ──
    text = input_func(prompt).strip()
    if not text:
        return None
    if text.lower() in {"quit", "exit"}:
        return MoveCommand(command_type="quit")

    try:
        return parse_command(text, board=board)
    except ValueError as exc:
        print(f"error: {exc}", file=sys.stderr)
        return None
