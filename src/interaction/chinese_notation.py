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

BLACK_CHINESE_FILES = {
    "一": 8,
    "二": 7,
    "三": 6,
    "四": 5,
    "五": 4,
    "六": 3,
    "七": 2,
    "八": 1,
    "九": 0,
}

_NUMERAL_VALUE = {
    "一": 1,
    "二": 2,
    "三": 3,
    "四": 4,
    "五": 5,
    "六": 6,
    "七": 7,
    "八": 8,
    "九": 9,
}

PIECE_TYPES = {
    "车": PieceType.ROOK,
    "马": PieceType.HORSE,
    "炮": PieceType.CANNON,
}

BLACK_PIECE_TYPES = {
    "車": PieceType.ROOK,
    "馬": PieceType.HORSE,
    "砲": PieceType.CANNON,
    "将": PieceType.GENERAL,
    "士": PieceType.ADVISOR,
    "象": PieceType.ELEPHANT,
    "卒": PieceType.SOLDIER,
}

CHINESE_ACTIONS = {"平", "进", "進", "退"}


def parse_chinese_move(text: str, board: BoardState, side: str = "red") -> MoveCommand:
    """Parse Chinese chess notation for both red and black sides.

    Supported forms:
    - Red:  车二平七 / 炮五进四 / 前车二平七 / 车二退一 / 马二进三
    - Black: 車一平九 / 砲五進四 / 前車二平七
    - Action: 平 (horizontal), 进/進 (advance), 退 (retreat)
    - 前/后 prefix for disambiguation on same-file same-type pieces.
    """
    if side not in {"red", "black"}:
        raise ValueError("side must be 'red' or 'black'")

    prefix = ""
    move_text = text.strip()
    if move_text[:1] in {"前", "后"}:
        prefix = move_text[0]
        move_text = move_text[1:]

    if len(move_text) != 4:
        raise ValueError("Chinese notation must look like 车二平七 or 炮五进四")

    piece_char, source_file_char, action_char, target_char = move_text

    if side == "black":
        files_map = BLACK_CHINESE_FILES
        piece_types_map = BLACK_PIECE_TYPES
        color = PieceColor.BLACK
    else:
        files_map = CHINESE_FILES
        piece_types_map = PIECE_TYPES
        color = PieceColor.RED

    if piece_char not in piece_types_map:
        raise ValueError(f"unsupported piece for {side} Chinese notation")
    if source_file_char not in files_map:
        raise ValueError("source file must be 一 to 九")
    if action_char not in CHINESE_ACTIONS:
        raise ValueError("only 平, 进, and 退 are supported")
    if target_char not in files_map:
        raise ValueError("target file/step must be 一 to 九")

    piece_type = piece_types_map[piece_char]
    source_col = files_map[source_file_char]
    piece = _select_piece(board, piece_type, color, source_col, prefix)
    from_col, from_row = _split_cell(piece.cell)

    if action_char == "平":
        target_col = files_map[target_char]
        to_cell = _join_cell(target_col, from_row)
    else:
        steps = _NUMERAL_VALUE[target_char]
        is_advance = action_char in {"进", "進"}
        if side == "black":
            # Black: advance toward red (row decreases), retreat (row increases)
            new_row = from_row - steps if is_advance else from_row + steps
        else:
            # Red: advance toward black (row increases), retreat (row decreases)
            new_row = from_row + steps if is_advance else from_row - steps
        to_cell = _join_cell(from_col, new_row)

    return MoveCommand(command_type="move", from_cell=piece.cell, to_cell=to_cell)


def _select_piece(board: BoardState, piece_type: PieceType, color: PieceColor,
                  source_col: int, prefix: str) -> Piece:
    candidates = [
        piece
        for piece in board.pieces.values()
        if piece.color == color and piece.kind == piece_type and _split_cell(piece.cell)[0] == source_col
    ]
    if not candidates:
        raise ValueError(f"no matching {color.value} piece found for Chinese notation")
    if len(candidates) == 1:
        return candidates[0]
    if prefix not in {"前", "后"}:
        raise ValueError("same-file same-type pieces are ambiguous; please specify 前/后")

    candidates.sort(key=lambda piece: _split_cell(piece.cell)[1])
    # Red: 前 = higher row (closer to opponent); Black: 前 = lower row (closer to opponent)
    if color == PieceColor.RED:
        return candidates[-1] if prefix == "前" else candidates[0]
    else:
        return candidates[0] if prefix == "前" else candidates[-1]


def _split_cell(cell: str) -> tuple[int, int]:
    col = ord(cell[0].upper()) - ord("A")
    row = int(cell[1:]) - 1
    return col, row


def _join_cell(col: int, row: int) -> str:
    if not 0 <= col <= 8 or not 0 <= row <= 9:
        raise ValueError("Chinese notation target is outside the board")
    return f"{chr(ord('A') + col)}{row + 1}"
