from __future__ import annotations

from src.common.types import BoardState, MoveCommand, Piece, PieceColor, PieceType


CHINESE_FILES = {
    "一": 0,
    "二": 1,
    "三": 2,
    "四": 3,
    "五": 4,
    "六": 5,
    "七": 6,
    "八": 7,
    "九": 8,
}

PIECE_TYPES = {
    "车": PieceType.ROOK,
    "炮": PieceType.CANNON,
}


def parse_chinese_move(text: str, board: BoardState, side: str = "red") -> MoveCommand:
    """Parse the small red-side Chinese notation subset used by this project.

    Supported demo forms:
    - 车二平七: red rook on file two moves horizontally to file seven.
    - 炮五进四: red cannon on file five advances four rows.
    - 前车二平七 / 后车二平七: disambiguate same-file same-type pieces.
    """
    if side != "red":
        raise ValueError("only red-side Chinese notation is supported")

    prefix = ""
    move_text = text.strip()
    if move_text[:1] in {"前", "后"}:
        prefix = move_text[0]
        move_text = move_text[1:]

    if len(move_text) != 4:
        raise ValueError("Chinese notation must look like 车二平七 or 炮五进四")

    piece_char, source_file_char, action_char, target_char = move_text
    if piece_char not in PIECE_TYPES:
        raise ValueError("only 车 and 炮 Chinese notation are supported")
    if source_file_char not in CHINESE_FILES:
        raise ValueError("source file must be 一 to 九")
    if action_char not in {"平", "进"}:
        raise ValueError("only 平 and 进 are supported")
    if target_char not in CHINESE_FILES:
        raise ValueError("target file/step must be 一 to 九")

    piece_type = PIECE_TYPES[piece_char]
    source_col = CHINESE_FILES[source_file_char]
    piece = _select_red_piece(board, piece_type, source_col, prefix)
    from_col, from_row = _split_cell(piece.cell)

    if action_char == "平":
        target_col = CHINESE_FILES[target_char]
        to_cell = _join_cell(target_col, from_row)
    else:
        steps = CHINESE_FILES[target_char] + 1
        to_cell = _join_cell(from_col, from_row + steps)

    return MoveCommand(command_type="move", from_cell=piece.cell, to_cell=to_cell)


def _select_red_piece(board: BoardState, piece_type: PieceType, source_col: int, prefix: str) -> Piece:
    candidates = [
        piece
        for piece in board.pieces.values()
        if piece.color == PieceColor.RED and piece.kind == piece_type and _split_cell(piece.cell)[0] == source_col
    ]
    if not candidates:
        raise ValueError("no matching red piece found for Chinese notation")
    if len(candidates) == 1:
        return candidates[0]
    if prefix not in {"前", "后"}:
        raise ValueError("same-file same-type pieces are ambiguous; please specify 前/后")

    candidates.sort(key=lambda piece: _split_cell(piece.cell)[1])
    return candidates[-1] if prefix == "前" else candidates[0]


def _split_cell(cell: str) -> tuple[int, int]:
    col = ord(cell[0].upper()) - ord("A")
    row = int(cell[1:]) - 1
    return col, row


def _join_cell(col: int, row: int) -> str:
    if not 0 <= col <= 8 or not 0 <= row <= 9:
        raise ValueError("Chinese notation target is outside the board")
    return f"{chr(ord('A') + col)}{row + 1}"
