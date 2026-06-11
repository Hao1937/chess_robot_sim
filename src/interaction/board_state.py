from __future__ import annotations

from src.common.types import BoardState, LogicalAction, MoveCommand, Piece, PieceColor, PieceType


def create_initial_board() -> BoardState:
    """Create a small demo board; teammates can expand it to full Chinese chess."""
    return BoardState(
        pieces={
            "A1": Piece(piece_id="red_rook_1", kind=PieceType.ROOK, color=PieceColor.RED, cell="A1"),
            "B1": Piece(piece_id="black_horse_1", kind=PieceType.HORSE, color=PieceColor.BLACK, cell="B1"),
            "C1": Piece(piece_id="red_cannon_1", kind=PieceType.CANNON, color=PieceColor.RED, cell="C1"),
        }
    )


def make_logical_actions(board: BoardState, command: MoveCommand) -> list[LogicalAction]:
    """Convert a legal move command into logical pick/place actions."""
    if command.command_type == "reset":
        return make_reset_actions(board)
    if command.command_type != "move":
        return [LogicalAction(action_type="safety_pause", cell="", piece_id=command.command_type)]

    moving_piece = board.pieces.get(command.from_cell)
    if moving_piece is None:
        raise ValueError(f"no piece at {command.from_cell}")

    actions: list[LogicalAction] = []
    target_piece = board.pieces.get(command.to_cell)
    if target_piece is not None and target_piece.color == moving_piece.color:
        raise ValueError("target cell contains friendly piece")

    if target_piece is not None and target_piece.color != moving_piece.color:
        captured_cell = _next_captured_cell(board, target_piece.color)
        actions.extend([
            LogicalAction(action_type="pick", cell=command.to_cell, piece_id=target_piece.piece_id),
            LogicalAction(action_type="place", cell=captured_cell, piece_id=target_piece.piece_id),
        ])

    actions.extend([
        LogicalAction(action_type="pick", cell=command.from_cell, piece_id=moving_piece.piece_id),
        LogicalAction(action_type="place", cell=command.to_cell, piece_id=moving_piece.piece_id),
    ])
    return actions


def make_reset_actions(board: BoardState) -> list[LogicalAction]:
    """Return a simple reset sequence based on piece ids and current cells."""
    actions: list[LogicalAction] = []
    for current_cell, piece in sorted(board.pieces.items()):
        home_cell = _home_cell_for_piece(piece)
        if current_cell != home_cell:
            actions.append(LogicalAction(action_type="pick", cell=current_cell, piece_id=piece.piece_id))
            actions.append(LogicalAction(action_type="place", cell=home_cell, piece_id=piece.piece_id))
    return actions


def _next_captured_cell(board: BoardState, color: PieceColor) -> str:
    count = board.captured_counts.get(color, 0) + 1
    return f"CAPTURED_{color.name}_{count}"


def _home_cell_for_piece(piece: Piece) -> str:
    if piece.kind == PieceType.ROOK and piece.color == PieceColor.RED:
        return "A1"
    if piece.kind == PieceType.HORSE and piece.color == PieceColor.BLACK:
        return "B1"
    if piece.kind == PieceType.CANNON and piece.color == PieceColor.RED:
        return "C1"
    return piece.cell
