from __future__ import annotations

import queue
import time

from src.common.types import BoardState, MoveCommand, PieceColor, PieceType

# ── Piece character mapping for board display ──

PIECE_CHARS: dict[tuple[PieceColor, PieceType], str] = {
    (PieceColor.RED, PieceType.ROOK): '车',
    (PieceColor.RED, PieceType.HORSE): '馬',
    (PieceColor.RED, PieceType.CANNON): '炮',
    (PieceColor.RED, PieceType.GENERAL): '帥',
    (PieceColor.RED, PieceType.ADVISOR): '仕',
    (PieceColor.RED, PieceType.ELEPHANT): '相',
    (PieceColor.RED, PieceType.SOLDIER): '兵',
    (PieceColor.BLACK, PieceType.ROOK): '車',
    (PieceColor.BLACK, PieceType.HORSE): '馬',
    (PieceColor.BLACK, PieceType.CANNON): '炮',
    (PieceColor.BLACK, PieceType.GENERAL): '將',
    (PieceColor.BLACK, PieceType.ADVISOR): '士',
    (PieceColor.BLACK, PieceType.ELEPHANT): '象',
    (PieceColor.BLACK, PieceType.SOLDIER): '卒',
}

_NUM_COLS = 9
_NUM_ROWS = 10


class BoardGUI:
    """Matplotlib-based clickable Chinese chess board.

    First click selects a source cell (highlighted solid blue).
    While a cell is selected, a dashed hover rectangle follows the mouse
    to preview the target cell.  Second click (different cell) enqueues a
    MoveCommand.  Clicking the same cell again deselects.

    Usage::

        gui = BoardGUI(board)
        while True:
            cmd = gui.get_next_command()
            if cmd is not None:
                print(f"move from {cmd.from_cell} to {cmd.to_cell}")
    """

    # ── constructor ──

    def __init__(self, board: BoardState):
        self._ensure_matplotlib()
        import matplotlib.pyplot as _plt
        from matplotlib.patches import Rectangle as _Rectangle

        self._plt = _plt
        self._Rectangle = _Rectangle
        self._command_queue: queue.Queue[MoveCommand] = queue.Queue()
        self._selected_cell: str | None = None
        self._highlight_rect = None       # solid blue selection
        self._hover_rect = None           # dashed grey hover preview
        self._board = board
        self._window_open = True
        self._cid_click: int | None = None
        self._cid_motion: int | None = None
        self._cid_close: int | None = None
        self._last_draw_time = 0.0
        self._valid_targets: set[str] = set()
        self._valid_target_patches: list = []

        _plt.ion()
        self._fig, self._ax = _plt.subplots(figsize=(5.5, 6))
        try:
            self._fig.canvas.manager.set_window_title("Chinese Chess Board")
        except Exception:
            pass
        self._cid_click = self._fig.canvas.mpl_connect(
            'button_press_event', self._on_click,
        )
        self._cid_motion = self._fig.canvas.mpl_connect(
            'motion_notify_event', self._on_motion,
        )
        self._cid_close = self._fig.canvas.mpl_connect(
            'close_event', self._on_close,
        )
        self._draw(board)

    @staticmethod
    def _ensure_matplotlib() -> None:
        """Configure matplotlib with interactive backend + CJK font."""
        try:
            import matplotlib  # noqa: F401
        except ImportError:
            raise ImportError(
                "matplotlib is required for BoardGUI. "
                "Install with: pip install matplotlib"
            )

        # Force interactive GUI backend (not Agg)
        import matplotlib as _mpl
        _current = _mpl.get_backend()
        if _current.lower() in ("agg", "template", "svg", "pdf", "ps", "cairo"):
            _gui_backends = ("TkAgg", "Qt5Agg", "QtAgg", "GTK3Agg", "wxAgg")
            _switched = False
            for _backend in _gui_backends:
                try:
                    _mpl.use(_backend, force=True)
                    _switched = True
                    break
                except Exception:
                    continue
            if not _switched:
                raise ImportError(
                    "No interactive matplotlib backend available. "
                    "Install tkinter or PyQt5 for GUI support."
                )

        # CJK font setup to avoid tofu / missing-glyph warnings
        try:
            import matplotlib.font_manager as fm
            import matplotlib.pyplot as plt

            _CJK_CANDIDATES = [
                "Microsoft YaHei", "SimHei", "WenQuanYi Micro Hei",
                "Noto Sans CJK SC", "Source Han Sans SC", "PingFang SC",
                "Heiti SC", "STHeiti", "sans-serif",
            ]
            available = {f.name for f in fm.fontManager.ttflist}
            for name in _CJK_CANDIDATES:
                if name in available:
                    plt.rcParams["font.family"] = name
                    break
        except Exception:
            pass  # silent degrade — pieces may lack glyphs but won't crash

    # ── public API ──

    def get_next_command(self) -> MoveCommand | None:
        """Non-blocking poll: pump GUI events and return any queued command.

        Keeps the matplotlib window responsive by running its event loop for
        a short interval each call.  Returns None when no command is ready
        or the window was closed.
        """
        if not self._window_open:
            return None

        # Pump the GUI event loop so clicks / hover are processed.
        # 30 ms is long enough to catch a click, short enough not to stall
        # the PyBullet simulation noticeably.
        try:
            self._plt.pause(0.03)
        except Exception:
            pass

        try:
            return self._command_queue.get_nowait()
        except queue.Empty:
            return None

    def update_board(self, board: BoardState):
        """Redraw the board to reflect an updated BoardState."""
        if not self._window_open:
            return
        self._board = board
        self._selected_cell = None
        self._highlight_rect = None
        self._hover_rect = None
        if hasattr(self, '_valid_targets'):
            self._valid_targets.clear()
        self._clear_valid_targets()
        self._ax.clear()
        self._draw(board)
        self._fig.canvas.draw_idle()

    def set_status(self, text: str):
        """Show a status message on the board (e.g. '机械臂移动中...').

        An empty string clears the status. The message is drawn at the top of
        the board and immediately flushed to the GUI.
        """
        if not self._window_open:
            return
        # Remove any existing status text (stored as an attribute)
        old = getattr(self, '_status_text', None)
        if old is not None:
            try:
                old.remove()
            except Exception:
                pass
            self._status_text = None
        if text:
            self._status_text = self._ax.text(
                4.0, -0.8, text,
                fontsize=10, ha='center', va='center',
                color='#444444', alpha=0.8,
                bbox=dict(
                    boxstyle='round,pad=0.3',
                    facecolor='#ffffcc',
                    edgecolor='#cccccc',
                    linewidth=0.5,
                ),
            )
        self._fig.canvas.draw_idle()

    def close(self):
        """Close the board window and release resources."""
        self._window_open = False
        for cid in (self._cid_click, self._cid_motion, self._cid_close):
            if cid is not None:
                try:
                    self._fig.canvas.mpl_disconnect(cid)
                except Exception:
                    pass
        self._cid_click = None
        self._cid_motion = None
        self._cid_close = None
        try:
            self._plt.close(self._fig)
        except Exception:
            pass

    @property
    def is_open(self) -> bool:
        """Return True while the board window is displayed."""
        return self._window_open

    # ── drawing ──

    def _draw(self, board: BoardState) -> None:
        """Render the board grid, river, palace diagonals, pieces, and labels."""
        ax = self._ax
        ax.set_facecolor('#f5deb3')  # wheat

        # ── grid lines ──
        for row in range(_NUM_ROWS):
            ax.plot([0, _NUM_COLS - 1], [row, row],
                    color='black', linewidth=0.6)

        # outer vertical lines (full height)
        ax.plot([0, 0], [0, _NUM_ROWS - 1], color='black', linewidth=0.6)
        ax.plot([_NUM_COLS - 1, _NUM_COLS - 1], [0, _NUM_ROWS - 1],
                color='black', linewidth=0.6)

        # inner vertical lines (broken by river between rows 4 and 5)
        for col in range(1, _NUM_COLS - 1):
            ax.plot([col, col], [0, 4], color='black', linewidth=0.6)
            ax.plot([col, col], [5, _NUM_ROWS - 1],
                    color='black', linewidth=0.6)

        # ── river ──
        # 河界虚线（row=4 和 row=5 之间）
        ax.plot([0, 8], [4.5, 4.5], color='black', linewidth=0.6, linestyle='--', alpha=0.35)
        # 分两段居中文本
        ax.text(2.0, 4.5, '楚  河', fontsize=12, ha='center', va='center', color='black', alpha=0.5)
        ax.text(7.0, 4.5, '汉  界', fontsize=12, ha='center', va='center', color='black', alpha=0.5)

        # ── palace diagonals ──
        self._draw_diagonal(3, 0, 5, 2)
        self._draw_diagonal(5, 0, 3, 2)
        self._draw_diagonal(3, 7, 5, 9)
        self._draw_diagonal(5, 7, 3, 9)

        # ── pieces ──
        for piece in board.pieces.values():
            if piece.cell.startswith("CAPTURED_"):
                continue
            col, row = _cell_to_grid(piece.cell)
            if col is None or row is None:
                continue
            char = PIECE_CHARS.get(
                (piece.color, piece.kind),
                '?',
            )
            color = 'red' if piece.color == PieceColor.RED else 'black'
            ax.text(
                col, row, char,
                ha='center', va='center',
                fontsize=16, color=color,
                bbox=dict(
                    boxstyle='circle,pad=0.15',
                    facecolor='#f5deb3',
                    edgecolor='black',
                    linewidth=1.0,
                ),
            )

        # ── column labels (A-I) at bottom（棋子之后绘制，避免遮挡）──
        for col in range(_NUM_COLS):
            ax.text(col, -0.5, chr(ord('A') + col),
                    ha='center', va='center', fontsize=9, color='black',
                    zorder=12)

        # ── row labels (1-10) at left（棋子之后绘制，避免遮挡）──
        for row in range(_NUM_ROWS):
            ax.text(-0.5, row, str(row + 1),
                    ha='center', va='center', fontsize=9, color='black',
                    zorder=12)

        # ── selection highlight (solid blue) ──
        if self._selected_cell is not None:
            sc, sr = _cell_to_grid(self._selected_cell)
            if sc is not None and sr is not None:
                self._highlight_rect = self._Rectangle(
                    (sc - 0.46, sr - 0.46), 0.92, 0.92,
                    linewidth=2.5, edgecolor='blue',
                    facecolor='none', zorder=10,
                )
                ax.add_patch(self._highlight_rect)

        # ── valid target highlights (green dashed) ──
        self._draw_valid_targets()

        ax.set_xlim(-0.6, 9.5)
        ax.set_ylim(-0.6, 10.1)
        ax.set_aspect('equal')
        ax.axis('off')

    def _draw_diagonal(self, x1: int, y1: int, x2: int, y2: int) -> None:
        """Draw one palace diagonal segment."""
        self._ax.plot([x1, x2], [y1, y2], color='black', linewidth=0.6)

    # ── interaction ──

    def _on_click(self, event) -> None:
        """Handle mouse click on the board."""
        if event.xdata is None or event.ydata is None:
            return
        col = round(event.xdata)
        row = round(event.ydata)
        if not (0 <= col < _NUM_COLS and 0 <= row < _NUM_ROWS):
            return

        cell = _grid_to_cell(col, row)

        if self._selected_cell is None:
            # First click: select source
            self._selected_cell = cell
            self._draw_highlight(col, row)
            self._valid_targets = self._compute_valid_targets(self._board, cell)
            self._draw_valid_targets()
        elif self._selected_cell == cell:
            # Click same cell again: deselect
            self._selected_cell = None
            self._clear_highlight()
            self._clear_hover()
            if hasattr(self, '_valid_targets'):
                self._valid_targets.clear()
            self._clear_valid_targets()
        else:
            # Second click (different cell): enqueue command
            self._command_queue.put(
                MoveCommand(
                    command_type="move",
                    from_cell=self._selected_cell,
                    to_cell=cell,
                )
            )
            self._selected_cell = None
            self._clear_highlight()
            self._clear_hover()
            if hasattr(self, '_valid_targets'):
                self._valid_targets.clear()
            self._clear_valid_targets()

        self._fig.canvas.draw_idle()

    def _on_motion(self, event) -> None:
        """Update dashed hover rectangle on mouse movement."""
        if event.xdata is None or event.ydata is None:
            if self._hover_rect is not None:
                self._clear_hover()
                self._fig.canvas.draw_idle()
            return

        col = round(event.xdata)
        row = round(event.ydata)
        if not (0 <= col < _NUM_COLS and 0 <= row < _NUM_ROWS):
            self._clear_hover()
            self._fig.canvas.draw_idle()
            return

        # Skip hover over the currently selected cell
        if self._selected_cell is not None and (col, row) == _cell_to_grid(self._selected_cell):
            self._clear_hover()
            self._fig.canvas.draw_idle()
            return

        # Throttle redraws to ~20 fps
        now = time.monotonic()
        if now - self._last_draw_time < 0.05:
            return

        # Determine line style based on whether cell has a piece
        cell = _grid_to_cell(col, row)
        has_piece = cell in self._board.pieces
        if has_piece:
            self._draw_hover(col, row, linewidth=2.0, edgecolor='#555555', alpha=1.0)
        else:
            self._draw_hover(col, row, linewidth=1.2, edgecolor='#aaaaaa', alpha=0.5)

        self._last_draw_time = now
        self._fig.canvas.draw_idle()

    def _on_close(self, _event) -> None:
        """Handle window close event."""
        self._window_open = False

    def _draw_highlight(self, col: int, row: int) -> None:
        """Draw or move the solid blue selection highlight rectangle."""
        self._clear_highlight()
        self._highlight_rect = self._Rectangle(
            (col - 0.46, row - 0.46), 0.92, 0.92,
            linewidth=2.5, edgecolor='blue',
            facecolor='none', zorder=10,
        )
        self._ax.add_patch(self._highlight_rect)

    def _clear_highlight(self) -> None:
        """Remove the selection highlight."""
        if self._highlight_rect is not None:
            try:
                self._highlight_rect.remove()
            except Exception:
                pass
            self._highlight_rect = None

    def _draw_hover(self, col: int, row: int, linewidth: float = 1.8,
                    edgecolor: str = 'grey', alpha: float = 1.0) -> None:
        """Draw or move the dashed hover preview rectangle."""
        self._clear_hover()
        self._hover_rect = self._Rectangle(
            (col - 0.46, row - 0.46), 0.92, 0.92,
            linewidth=linewidth, edgecolor=edgecolor,
            facecolor='none', linestyle='--',
            zorder=9, alpha=alpha,
        )
        self._ax.add_patch(self._hover_rect)

    def _clear_hover(self) -> None:
        """Remove the hover preview."""
        if self._hover_rect is not None:
            try:
                self._hover_rect.remove()
            except Exception:
                pass
            self._hover_rect = None

    def _compute_valid_targets(self, board: BoardState, source_cell: str) -> set[str]:
        """Compute all valid target cells for a piece at source_cell."""
        from src.interaction.chess_rules import validate_move
        from src.common.types import MoveCommand

        valid: set[str] = set()
        for col in range(_NUM_COLS):
            for row in range(_NUM_ROWS):
                target = _grid_to_cell(col, row)
                if target == source_cell:
                    continue
                cmd = MoveCommand(command_type="move", from_cell=source_cell, to_cell=target)
                try:
                    result = validate_move(board, cmd)
                    if result.is_legal:
                        valid.add(target)
                except Exception:
                    pass
        return valid

    def _draw_valid_targets(self) -> None:
        """Draw green dashed circles on valid target cells."""
        self._clear_valid_targets()
        # 防御 __new__ 构建的 mock BoardGUI（测试用）
        if '_valid_target_patches' not in self.__dict__:
            self._valid_target_patches = []
        targets = getattr(self, '_valid_targets', set())
        for cell in targets:
            col, row = _cell_to_grid(cell)
            if col is None or row is None:
                continue
            rect = self._Rectangle(
                (col - 0.46, row - 0.46), 0.92, 0.92,
                linewidth=1.5, edgecolor='green',
                facecolor='none', linestyle='--',
                zorder=8, alpha=0.6,
            )
            self._ax.add_patch(rect)
            self._valid_target_patches.append(rect)

    def _clear_valid_targets(self) -> None:
        """Remove valid target highlight patches."""
        patches = getattr(self, '_valid_target_patches', None)
        if patches is None:
            return
        for patch in patches:
            try:
                patch.remove()
            except Exception:
                pass
        patches.clear()


# ── grid helpers ──


def _cell_to_grid(cell: str) -> tuple[int | None, int | None]:
    """Convert a board cell string like 'A1' to grid (col, row)."""
    if not cell or cell[0].upper() < 'A' or cell[0].upper() > 'I':
        return None, None
    col = ord(cell[0].upper()) - ord('A')
    try:
        row = int(cell[1:]) - 1
    except (ValueError, IndexError):
        return None, None
    if 0 <= col < _NUM_COLS and 0 <= row < _NUM_ROWS:
        return col, row
    return None, None


def _grid_to_cell(col: int, row: int) -> str:
    """Convert grid (col, row) to a board cell string like 'A1'."""
    return f"{chr(ord('A') + col)}{row + 1}"
