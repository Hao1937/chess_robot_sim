from __future__ import annotations

from src.common.types import BoardState, MoveCommand, PieceColor, PieceType, ValidationResult


def validate_move(board: BoardState, command: MoveCommand) -> ValidationResult:
    """Validate a simplified Chinese chess move.

    This is not a full chess referee. It only gates obvious invalid inputs and
    implements movement rules for all piece types.
    """
    if command.command_type != "move":
        return ValidationResult(True)
    if command.from_cell not in board.pieces:
        return ValidationResult(False, f"no piece at {command.from_cell}")
    if command.from_cell == command.to_cell:
        return ValidationResult(False, "from_cell and to_cell are the same")

    moving_piece = board.pieces[command.from_cell]
    target_piece = board.pieces.get(command.to_cell)
    if target_piece is not None and target_piece.color == moving_piece.color:
        return ValidationResult(False, "target cell contains friendly piece")

    if moving_piece.kind == PieceType.ROOK:
        return _validate_rook(command)
    if moving_piece.kind == PieceType.HORSE:
        return _validate_horse(command)
    if moving_piece.kind == PieceType.CANNON:
        return _validate_cannon(board, command)
    if moving_piece.kind == PieceType.GENERAL:
        return _validate_general(board, command)
    if moving_piece.kind == PieceType.ADVISOR:
        return _validate_advisor(board, command)
    if moving_piece.kind == PieceType.ELEPHANT:
        return _validate_elephant(board, command)
    if moving_piece.kind == PieceType.SOLDIER:
        return _validate_soldier(board, command)
    return ValidationResult(True, "basic validation only for this piece type")


def _validate_rook(command: MoveCommand) -> ValidationResult:
    from_col, from_row = _split_cell(command.from_cell)
    to_col, to_row = _split_cell(command.to_cell)
    if from_col == to_col or from_row == to_row:
        return ValidationResult(True)
    return ValidationResult(False, "rook must move in a straight line")


def _validate_horse(command: MoveCommand) -> ValidationResult:
    from_col, from_row = _split_cell(command.from_cell)
    to_col, to_row = _split_cell(command.to_cell)
    dc = abs(to_col - from_col)
    dr = abs(to_row - from_row)
    if (dc, dr) in {(1, 2), (2, 1)}:
        return ValidationResult(True)
    return ValidationResult(False, "horse must move in an L shape")


def _validate_cannon(board: BoardState, command: MoveCommand) -> ValidationResult:
    from_col, from_row = _split_cell(command.from_cell)
    to_col, to_row = _split_cell(command.to_cell)
    if from_col != to_col and from_row != to_row:
        return ValidationResult(False, "cannon must move in a straight line")
    blockers = _count_between(board, command.from_cell, command.to_cell)
    is_capture = command.to_cell in board.pieces
    if is_capture and blockers == 1:
        return ValidationResult(True)
    if not is_capture and blockers == 0:
        return ValidationResult(True)
    return ValidationResult(False, "cannon capture needs exactly one screen piece")


# ── General (帅/将): one step orthogonal within the 3x3 palace ──

def _validate_general(board: BoardState, command: MoveCommand) -> ValidationResult:
    from_col, from_row = _split_cell(command.from_cell)
    to_col, to_row = _split_cell(command.to_cell)
    dc = abs(to_col - from_col)
    dr = abs(to_row - from_row)

    # Must move exactly one step orthogonally
    if not ((dc == 1 and dr == 0) or (dc == 0 and dr == 1)):
        return ValidationResult(False, "general must move one step orthogonally")

    # Must stay within palace
    piece = board.pieces[command.from_cell]
    if not _in_palace(to_col, to_row, piece.color):
        return ValidationResult(False, "general must stay within the palace")

    return ValidationResult(True)


# ── Advisor (仕/士): one step diagonally within the 3x3 palace ──

def _validate_advisor(board: BoardState, command: MoveCommand) -> ValidationResult:
    from_col, from_row = _split_cell(command.from_cell)
    to_col, to_row = _split_cell(command.to_cell)
    dc = abs(to_col - from_col)
    dr = abs(to_row - from_row)

    # Must move exactly one step diagonally
    if not (dc == 1 and dr == 1):
        return ValidationResult(False, "advisor must move one step diagonally")

    # Must stay within palace
    piece = board.pieces[command.from_cell]
    if not _in_palace(to_col, to_row, piece.color):
        return ValidationResult(False, "advisor must stay within the palace")

    return ValidationResult(True)


# ── Elephant (相/象): 田字 diagonal (2x2), cannot cross river, blocked by eye ──

def _validate_elephant(board: BoardState, command: MoveCommand) -> ValidationResult:
    from_col, from_row = _split_cell(command.from_cell)
    to_col, to_row = _split_cell(command.to_cell)
    dc = abs(to_col - from_col)
    dr = abs(to_row - from_row)

    # Must move in a 田 shape: 2 steps in each direction
    if not (dc == 2 and dr == 2):
        return ValidationResult(False, "elephant must move in a 田 shape (2x2 diagonal)")

    # Cannot cross river
    piece = board.pieces[command.from_cell]
    if piece.color == PieceColor.RED and to_row >= 5:
        return ValidationResult(False, "red elephant cannot cross the river")
    if piece.color == PieceColor.BLACK and to_row <= 4:
        return ValidationResult(False, "black elephant cannot cross the river")

    # Check elephant eye (midpoint between from and to)
    eye_col = (from_col + to_col) // 2
    eye_row = (from_row + to_row) // 2
    eye_cell = _join_cell(eye_col, eye_row)
    if eye_cell in board.pieces:
        return ValidationResult(False, "elephant eye is blocked")

    return ValidationResult(True)


# ── Soldier (兵/卒): forward only before river; forward + sideways after ──

def _validate_soldier(board: BoardState, command: MoveCommand) -> ValidationResult:
    from_col, from_row = _split_cell(command.from_cell)
    to_col, to_row = _split_cell(command.to_cell)
    dc = to_col - from_col
    dr = to_row - from_row

    # Must move exactly one step
    if abs(dc) + abs(dr) != 1:
        return ValidationResult(False, "soldier must move exactly one step")

    piece = board.pieces[command.from_cell]
    if piece.color == PieceColor.RED:
        # red cannot retreat (moves toward higher row only)
        if dr < 0:
            return ValidationResult(False, "soldier cannot retreat")
        has_crossed = from_row >= 5
    else:
        # black cannot retreat (moves toward lower row only)
        if dr > 0:
            return ValidationResult(False, "soldier cannot retreat")
        has_crossed = from_row <= 4

    # Before crossing river, only forward allowed (dc must be 0)
    if not has_crossed and dc != 0:
        return ValidationResult(False, "soldier cannot move sideways before crossing the river")

    return ValidationResult(True)


# ── Helpers ──

def _in_palace(col: int, row: int, color: PieceColor) -> bool:
    """Check whether (col, row) is inside the 3x3 palace for the given color."""
    if not (3 <= col <= 5):
        return False
    if color == PieceColor.RED:
        return 0 <= row <= 2
    else:
        return 7 <= row <= 9


def _count_between(board: BoardState, from_cell: str, to_cell: str) -> int:
    from_col, from_row = _split_cell(from_cell)
    to_col, to_row = _split_cell(to_cell)
    count = 0
    if from_col == to_col:
        start, end = sorted((from_row, to_row))
        cells = [_join_cell(from_col, row) for row in range(start + 1, end)]
    else:
        start, end = sorted((from_col, to_col))
        cells = [_join_cell(col, from_row) for col in range(start + 1, end)]
    for cell in cells:
        if cell in board.pieces:
            count += 1
    return count


def _split_cell(cell: str) -> tuple[int, int]:
    col = ord(cell[0].upper()) - ord("A")
    row = int(cell[1:]) - 1
    return col, row


def _join_cell(col: int, row: int) -> str:
    return f"{chr(ord('A') + col)}{row + 1}"
