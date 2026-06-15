from __future__ import annotations

import re
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from src.common.types import BoardState

from src.common.types import MoveCommand


_CELL_PATTERN = re.compile(r"^[A-I](?:[1-9]|10)$")

# 中文记谱特征字符——用于快速检测是否为中文走法命令
_CHINESE_NOTATION_RE = re.compile(r"[一-鿿]")


def parse_command(command_text: str, board: BoardState | None = None) -> MoveCommand:
    """将用户输入解析为 MoveCommand，同时支持坐标走法和中文记谱。

    坐标走法:
      - "A1 B1"  走子
      - "reset"  复位
      - "hand_on" / "hand_off"  人手安全区
      - "obstacle_mode 1" / "obstacle_mode 2" / "obstacle_mode 3"  障碍物预设

    中文记谱（需提供 board 参数）:
      - "车二平七"  / "炮五进四"  / "马二进三"  红方
      - "車一平九"  / "砲五進四"               黑方
      - "前车二平七" / "后炮五进四"            同列同类歧义消解
    """
    text = command_text.strip()
    if not text:
        raise ValueError("empty command")

    parts = text.split()
    lowered = [part.lower() for part in parts]

    # ── 英文关键字命令 ──
    if len(parts) == 1 and lowered[0] in {"reset", "hand_on", "hand_off"}:
        return MoveCommand(command_type=lowered[0])

    if len(parts) == 2 and lowered[0] == "obstacle_mode":
        if parts[1] not in {"1", "2", "3"}:
            raise ValueError("obstacle_mode must be 1, 2, or 3")
        return MoveCommand(command_type="obstacle_mode", mode=f"mode_{parts[1]}")

    # ── quit / exit ──
    if len(parts) == 1 and lowered[0] in {"quit", "exit"}:
        return MoveCommand(command_type="quit")

    # ── 中文记谱 ──
    if _CHINESE_NOTATION_RE.search(text):
        if board is None:
            raise ValueError(
                "Chinese notation requires board context; "
                "use 'A1 B1' coordinate format or provide a board"
            )
        from src.interaction.chinese_notation import parse_chinese_move
        return parse_chinese_move(text, board)

    # ── 坐标走法 "A1 B1" ──
    if len(parts) != 2:
        raise ValueError(
            "command must look like 'A1 B1' (coordinates) "
            "or '车二平七' (Chinese notation)"
        )

    from_cell = parts[0].upper()
    to_cell = parts[1].upper()
    if not _is_valid_cell(from_cell) or not _is_valid_cell(to_cell):
        raise ValueError("move cells must be board coordinates like A1 or I10")

    return MoveCommand(command_type="move", from_cell=from_cell, to_cell=to_cell)


def _is_valid_cell(cell: str) -> bool:
    return bool(_CELL_PATTERN.fullmatch(cell))
