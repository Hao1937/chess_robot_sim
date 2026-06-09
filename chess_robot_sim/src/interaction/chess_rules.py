from __future__ import annotations

from src.common.types import BoardState, MoveCommand, PieceType, ValidationResult


def validate_move(board: BoardState, command: MoveCommand) -> ValidationResult:
    """Validate a simplified Chinese chess move.

    This is not a full chess referee. It only gates obvious invalid inputs and
    implements demo rules for rook, horse, and cannon.
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
