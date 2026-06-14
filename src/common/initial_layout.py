from __future__ import annotations

from src.common.types import PieceColor, PieceType


INITIAL_PIECE_LAYOUT: tuple[tuple[str, str, PieceType, PieceColor, str], ...] = (
    ("A1", "red_rook_1", PieceType.ROOK, PieceColor.RED, "\u8eca"),
    ("B1", "red_horse_1", PieceType.HORSE, PieceColor.RED, "\u99ac"),
    ("C1", "red_elephant_1", PieceType.ELEPHANT, PieceColor.RED, "\u76f8"),
    ("D1", "red_advisor_1", PieceType.ADVISOR, PieceColor.RED, "\u4ed5"),
    ("E1", "red_general", PieceType.GENERAL, PieceColor.RED, "\u5e25"),
    ("F1", "red_advisor_2", PieceType.ADVISOR, PieceColor.RED, "\u4ed5"),
    ("G1", "red_elephant_2", PieceType.ELEPHANT, PieceColor.RED, "\u76f8"),
    ("H1", "red_horse_2", PieceType.HORSE, PieceColor.RED, "\u99ac"),
    ("I1", "red_rook_2", PieceType.ROOK, PieceColor.RED, "\u8eca"),
    ("B3", "red_cannon_1", PieceType.CANNON, PieceColor.RED, "\u70ae"),
    ("H3", "red_cannon_2", PieceType.CANNON, PieceColor.RED, "\u70ae"),
    ("A4", "red_soldier_1", PieceType.SOLDIER, PieceColor.RED, "\u5175"),
    ("C4", "red_soldier_2", PieceType.SOLDIER, PieceColor.RED, "\u5175"),
    ("E4", "red_soldier_3", PieceType.SOLDIER, PieceColor.RED, "\u5175"),
    ("G4", "red_soldier_4", PieceType.SOLDIER, PieceColor.RED, "\u5175"),
    ("I4", "red_soldier_5", PieceType.SOLDIER, PieceColor.RED, "\u5175"),
    ("A10", "black_rook_1", PieceType.ROOK, PieceColor.BLACK, "\u8eca"),
    ("B10", "black_horse_1", PieceType.HORSE, PieceColor.BLACK, "\u99ac"),
    ("C10", "black_elephant_1", PieceType.ELEPHANT, PieceColor.BLACK, "\u8c61"),
    ("D10", "black_advisor_1", PieceType.ADVISOR, PieceColor.BLACK, "\u58eb"),
    ("E10", "black_general", PieceType.GENERAL, PieceColor.BLACK, "\u5c07"),
    ("F10", "black_advisor_2", PieceType.ADVISOR, PieceColor.BLACK, "\u58eb"),
    ("G10", "black_elephant_2", PieceType.ELEPHANT, PieceColor.BLACK, "\u8c61"),
    ("H10", "black_horse_2", PieceType.HORSE, PieceColor.BLACK, "\u99ac"),
    ("I10", "black_rook_2", PieceType.ROOK, PieceColor.BLACK, "\u8eca"),
    ("B8", "black_cannon_1", PieceType.CANNON, PieceColor.BLACK, "\u70ae"),
    ("H8", "black_cannon_2", PieceType.CANNON, PieceColor.BLACK, "\u70ae"),
    ("A7", "black_soldier_1", PieceType.SOLDIER, PieceColor.BLACK, "\u5352"),
    ("C7", "black_soldier_2", PieceType.SOLDIER, PieceColor.BLACK, "\u5352"),
    ("E7", "black_soldier_3", PieceType.SOLDIER, PieceColor.BLACK, "\u5352"),
    ("G7", "black_soldier_4", PieceType.SOLDIER, PieceColor.BLACK, "\u5352"),
    ("I7", "black_soldier_5", PieceType.SOLDIER, PieceColor.BLACK, "\u5352"),
)

INITIAL_HOME_CELLS: dict[str, str] = {
    piece_id: cell for cell, piece_id, _kind, _color, _label in INITIAL_PIECE_LAYOUT
}
