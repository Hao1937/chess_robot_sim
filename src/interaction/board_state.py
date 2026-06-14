from __future__ import annotations

from src.common.initial_layout import INITIAL_HOME_CELLS, INITIAL_PIECE_LAYOUT
from src.common.types import BoardState, LogicalAction, MoveCommand, Piece, PieceColor


def create_initial_board() -> BoardState:
    """Create the full initial Chinese chess board shared with the simulator."""
    return BoardState(
        pieces={
            cell: Piece(piece_id=piece_id, kind=kind, color=color, cell=cell)
            for cell, piece_id, kind, color, _label in INITIAL_PIECE_LAYOUT
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
    return INITIAL_HOME_CELLS.get(piece.piece_id, piece.cell)
