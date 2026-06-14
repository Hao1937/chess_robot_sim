from __future__ import annotations

import math
from pathlib import Path
import tempfile

from src.common.config import DEFAULT_CONFIG, Config
from src.common.initial_layout import INITIAL_PIECE_LAYOUT
from src.common.types import LogicalAction, Obstacle, OperationResult, SceneHandle
from src.planning.chessboard_mapping import cell_to_world
from src.simulation._runtime import RUNTIME, clear_scene_bodies, ensure_client, p, project_root


_INITIAL_PIECES = tuple(
    (cell, piece_id, color.value, label)
    for cell, piece_id, _kind, color, label in INITIAL_PIECE_LAYOUT
)

_BOARD_LINE_COLOR = (0.18, 0.10, 0.04)
_BOARD_TEXT_COLOR = (0.24, 0.11, 0.03)
_PIECE_WOOD_COLOR = (0.78, 0.56, 0.30, 1.0)

_RED_TEXT_COLOR = (0.75, 0.02, 0.02)
_BLACK_TEXT_COLOR = (0.02, 0.02, 0.02)
_WAYPOINT_DEBUG_IDS: list[int] = []
_WAYPOINT_MARKER_BODY_IDS: list[int] = []


def build_scene(config: Config = DEFAULT_CONFIG, obstacle_mode: str = "mode_1") -> SceneHandle:
    """Build the table, chessboard, pieces, captured area, and obstacles."""
    obstacles = build_obstacle_preset(obstacle_mode, config)
    if p is None:
        return SceneHandle(
            board_id=100,
            piece_ids={"A1": 201, "B1": 202, "C1": 203},
            obstacles=obstacles,
        )

    client_id = ensure_client()
    if client_id is None:
        return SceneHandle(
            board_id=100,
            piece_ids={"A1": 201, "B1": 202, "C1": 203},
            obstacles=obstacles,
        )

    clear_debug_visuals()
    clear_scene_bodies()
    _create_table(config, client_id)
    board_id = _create_board(config, client_id)
    _draw_chinese_chess_board(config, client_id)
    _create_captured_area(config, client_id)

    piece_ids: dict[str, int] = {}
    for cell, piece_id, color, label in _INITIAL_PIECES:
        body_id = _create_piece(cell, piece_id, color, label, config, client_id)
        piece_ids[cell] = body_id

    for obstacle in obstacles:
        _create_obstacle_body(obstacle, client_id)

    _set_camera(config, client_id)
    return SceneHandle(board_id=board_id, piece_ids=piece_ids, obstacles=obstacles)


def move_piece_to_cell(piece_id: str, cell: str, config: Config = DEFAULT_CONFIG) -> bool:
    """Move a simulated piece body to a board or captured-area cell."""
    if p is None or RUNTIME.client_id is None:
        return False
    body_id = RUNTIME.piece_body_ids.get(piece_id)
    if body_id is None:
        return False

    x, y, z = cell_to_world(cell, config)
    p.resetBasePositionAndOrientation(
        body_id,
        (x, y, z + config.piece_height / 2.0),
        (0.0, 0.0, 0.0, 1.0),
        physicsClientId=RUNTIME.client_id,
    )
    old_cell = RUNTIME.piece_cells.get(piece_id)
    if old_cell:
        RUNTIME.piece_ids_by_cell.pop(old_cell, None)
    RUNTIME.piece_cells[piece_id] = cell
    RUNTIME.piece_ids_by_cell[cell] = piece_id
    return True


def move_piece_to_captured_area(piece_id: str, captured_cell: str, config: Config = DEFAULT_CONFIG) -> OperationResult:
    """Move a known piece to a captured-area virtual cell supplied by A/C."""
    if not captured_cell.startswith("CAPTURED_"):
        return OperationResult(False, f"not a captured-area cell: {captured_cell}")
    if not move_piece_to_cell(piece_id, captured_cell, config):
        return OperationResult(False, f"could not move {piece_id} to {captured_cell}")
    return OperationResult(True, f"moved {piece_id} to {captured_cell}")


def apply_logical_action(action: LogicalAction, config: Config = DEFAULT_CONFIG) -> OperationResult:
    """Apply one A/C logical action to simulation visuals without deciding rules."""
    if action.action_type == "pick":
        return OperationResult(True, f"pick visual unchanged for {action.piece_id or action.cell}")
    if action.action_type != "place":
        return OperationResult(True, f"ignored non-place simulation action: {action.action_type}")

    piece_id = action.piece_id or RUNTIME.piece_ids_by_cell.get(action.cell, "")
    if not piece_id:
        return OperationResult(False, f"place action has no known piece for {action.cell}")
    if action.cell.startswith("CAPTURED_"):
        return move_piece_to_captured_area(piece_id, action.cell, config)
    if not move_piece_to_cell(piece_id, action.cell, config):
        return OperationResult(False, f"could not move {piece_id} to {action.cell}")
    return OperationResult(True, f"moved {piece_id} to {action.cell}")


def apply_logical_actions(actions: list[LogicalAction], config: Config = DEFAULT_CONFIG) -> list[OperationResult]:
    """Apply an A/C-generated pick/place sequence to the visible scene."""
    return [apply_logical_action(action, config) for action in actions]


def clear_debug_visuals() -> None:
    """Remove dynamic waypoint/path visuals while keeping board labels intact."""
    if p is None or RUNTIME.client_id is None or not p.isConnected(RUNTIME.client_id):
        _WAYPOINT_DEBUG_IDS.clear()
        _WAYPOINT_MARKER_BODY_IDS.clear()
        return
    for debug_id in _WAYPOINT_DEBUG_IDS:
        try:
            p.removeUserDebugItem(debug_id, physicsClientId=RUNTIME.client_id)
        except Exception:
            pass
    for body_id in _WAYPOINT_MARKER_BODY_IDS:
        try:
            p.removeBody(body_id, physicsClientId=RUNTIME.client_id)
        except Exception:
            pass
        if body_id in RUNTIME.scene_body_ids:
            RUNTIME.scene_body_ids.remove(body_id)
    _WAYPOINT_DEBUG_IDS.clear()
    _WAYPOINT_MARKER_BODY_IDS.clear()


def draw_waypoints(
    points_xyz: list[tuple[float, float, float]],
    *,
    clear_existing: bool = True,
    color_rgb: tuple[float, float, float] = (0.0, 0.55, 1.0),
    point_radius: float = 0.006,
) -> list[int]:
    """Draw C-provided Cartesian waypoints as a path line and small markers."""
    if clear_existing:
        clear_debug_visuals()
    if p is None:
        return []
    client_id = ensure_client()
    if client_id is None or not points_xyz:
        return []

    created_ids: list[int] = []
    for start, end in zip(points_xyz, points_xyz[1:]):
        debug_id = p.addUserDebugLine(
            start,
            end,
            lineColorRGB=color_rgb,
            lineWidth=2.5,
            lifeTime=0,
            physicsClientId=client_id,
        )
        _WAYPOINT_DEBUG_IDS.append(debug_id)
        created_ids.append(debug_id)

    visual_id = p.createVisualShape(
        p.GEOM_SPHERE,
        radius=point_radius,
        rgbaColor=(*color_rgb, 0.95),
        physicsClientId=client_id,
    )
    for point in points_xyz:
        body_id = p.createMultiBody(
            baseMass=0.0,
            baseCollisionShapeIndex=-1,
            baseVisualShapeIndex=visual_id,
            basePosition=point,
            physicsClientId=client_id,
        )
        RUNTIME.scene_body_ids.append(body_id)
        _WAYPOINT_MARKER_BODY_IDS.append(body_id)
        created_ids.append(body_id)
    return created_ids


def set_human_safety_zone(hand_present: bool, config: Config = DEFAULT_CONFIG) -> int | None:
    """Show or hide the dynamic human-hand safety zone in the PyBullet scene."""
    if p is None:
        return None
    client_id = ensure_client()
    if client_id is None:
        return None

    if RUNTIME.human_zone_body_id is not None:
        try:
            p.removeBody(RUNTIME.human_zone_body_id, physicsClientId=client_id)
        except Exception:
            pass
        if RUNTIME.human_zone_body_id in RUNTIME.scene_body_ids:
            RUNTIME.scene_body_ids.remove(RUNTIME.human_zone_body_id)
        RUNTIME.human_zone_body_id = None

    if not hand_present:
        return None

    center, radius, length, orientation_rpy = _human_safety_zone_geometry(config)
    visual_id = p.createVisualShape(
        p.GEOM_CYLINDER,
        radius=radius,
        length=length,
        rgbaColor=(1.0, 0.0, 0.0, 0.25),
        physicsClientId=client_id,
    )
    collision_id = p.createCollisionShape(
        p.GEOM_CYLINDER,
        radius=radius,
        height=length,
        physicsClientId=client_id,
    )
    body_id = p.createMultiBody(
        baseMass=0.0,
        baseCollisionShapeIndex=collision_id,
        baseVisualShapeIndex=visual_id,
        basePosition=center,
        baseOrientation=p.getQuaternionFromEuler(orientation_rpy),
        physicsClientId=client_id,
    )
    RUNTIME.human_zone_body_id = body_id
    RUNTIME.scene_body_ids.append(body_id)
    return body_id


def _human_safety_zone_geometry(
    config: Config = DEFAULT_CONFIG,
) -> tuple[tuple[float, float, float], float, float, tuple[float, float, float]]:
    """Return center, radius, length, and orientation for the horizontal hand zone."""
    return (
        config.human_hand_zone_center,
        config.human_hand_zone_radius,
        config.human_hand_zone_length,
        (0.0, math.pi / 2.0, 0.0),
    )


def _create_table(config: Config, client_id: int) -> int:
    x0, y0, _ = config.board_origin
    board_width = config.board_cols * config.cell_size
    board_depth = config.board_rows * config.cell_size
    center = (
        x0 + (config.board_cols - 1) * config.cell_size / 2.0,
        y0 + (config.board_rows - 1) * config.cell_size / 2.0,
        config.z_board - 0.045,
    )
    half_extents = (board_width / 2.0 + config.cell_size, board_depth / 2.0 + config.cell_size, 0.025)
    visual_id = p.createVisualShape(
        p.GEOM_BOX,
        halfExtents=half_extents,
        rgbaColor=(0.62, 0.50, 0.38, 1.0),
        physicsClientId=client_id,
    )
    collision_id = p.createCollisionShape(
        p.GEOM_BOX,
        halfExtents=half_extents,
        physicsClientId=client_id,
    )
    body_id = p.createMultiBody(
        baseMass=0.0,
        baseCollisionShapeIndex=collision_id,
        baseVisualShapeIndex=visual_id,
        basePosition=center,
        physicsClientId=client_id,
    )
    RUNTIME.scene_body_ids.append(body_id)
    return body_id


def _create_board(config: Config, client_id: int) -> int:
    board_width = config.board_cols * config.cell_size
    board_depth = config.board_rows * config.cell_size
    thickness = 0.006
    x0, y0, _ = config.board_origin
    center = (
        x0 + (config.board_cols - 1) * config.cell_size / 2.0,
        y0 + (config.board_rows - 1) * config.cell_size / 2.0,
        config.z_board - thickness / 2.0,
    )
    visual_id = p.createVisualShape(
        p.GEOM_BOX,
        halfExtents=(board_width / 2.0, board_depth / 2.0, thickness / 2.0),
        rgbaColor=(0.93, 0.88, 0.74, 1.0),
        physicsClientId=client_id,
    )
    collision_id = p.createCollisionShape(
        p.GEOM_BOX,
        halfExtents=(board_width / 2.0, board_depth / 2.0, thickness / 2.0),
        physicsClientId=client_id,
    )
    body_id = p.createMultiBody(
        baseMass=0.0,
        baseCollisionShapeIndex=collision_id,
        baseVisualShapeIndex=visual_id,
        basePosition=center,
        physicsClientId=client_id,
    )
    RUNTIME.scene_body_ids.append(body_id)
    return body_id


def _draw_chinese_chess_board(config: Config, client_id: int) -> None:
    x0, y0, _ = config.board_origin
    z = config.z_board + 0.003
    x_max = x0 + (config.board_cols - 1) * config.cell_size
    y_max = y0 + (config.board_rows - 1) * config.cell_size

    # Horizontal rank lines run across the whole board.
    for row in range(config.board_rows):
        y = y0 + row * config.cell_size
        _add_board_line((x0, y, z), (x_max, y, z), client_id)

    # File lines are broken at the river, except for the two outer borders.
    for col in range(config.board_cols):
        x = x0 + col * config.cell_size
        if col in {0, config.board_cols - 1}:
            _add_board_line((x, y0, z), (x, y_max, z), client_id)
        else:
            _add_board_line((x, y0, z), (x, y0 + 4 * config.cell_size, z), client_id)
            _add_board_line((x, y0 + 5 * config.cell_size, z), (x, y_max, z), client_id)

    _draw_outer_border(config, client_id, z)
    _draw_palaces(config, client_id, z)
    _draw_position_marks(config, client_id, z)
    _create_river_labels(config, client_id, z + 0.002)


def _add_board_line(
    start_xyz: tuple[float, float, float],
    end_xyz: tuple[float, float, float],
    client_id: int,
    width: float = 1.6,
) -> None:
    sx, sy, sz = start_xyz
    ex, ey, ez = end_xyz
    dx = ex - sx
    dy = ey - sy
    length = (dx * dx + dy * dy) ** 0.5
    if length <= 0.0:
        return

    line_width = width * 0.0015
    line_height = 0.0015
    visual_id = p.createVisualShape(
        p.GEOM_BOX,
        halfExtents=(length / 2.0, line_width / 2.0, line_height / 2.0),
        rgbaColor=(*_BOARD_LINE_COLOR, 1.0),
        physicsClientId=client_id,
    )
    yaw = math.atan2(dy, dx)
    body_id = p.createMultiBody(
        baseMass=0.0,
        baseCollisionShapeIndex=-1,
        baseVisualShapeIndex=visual_id,
        basePosition=((sx + ex) / 2.0, (sy + ey) / 2.0, (sz + ez) / 2.0),
        baseOrientation=p.getQuaternionFromEuler((0.0, 0.0, yaw)),
        physicsClientId=client_id,
    )
    RUNTIME.scene_body_ids.append(body_id)


def _draw_outer_border(config: Config, client_id: int, z: float) -> None:
    x0, y0, _ = config.board_origin
    x_max = x0 + (config.board_cols - 1) * config.cell_size
    y_max = y0 + (config.board_rows - 1) * config.cell_size
    inset = config.cell_size * 0.11
    corners = [
        (x0 - inset, y0 - inset, z),
        (x_max + inset, y0 - inset, z),
        (x_max + inset, y_max + inset, z),
        (x0 - inset, y_max + inset, z),
    ]
    for start, end in zip(corners, corners[1:] + corners[:1]):
        _add_board_line(start, end, client_id, width=2.6)


def _draw_palaces(config: Config, client_id: int, z: float) -> None:
    x0, y0, _ = config.board_origin
    c = config.cell_size
    _add_board_line((x0 + 3 * c, y0, z), (x0 + 5 * c, y0 + 2 * c, z), client_id)
    _add_board_line((x0 + 5 * c, y0, z), (x0 + 3 * c, y0 + 2 * c, z), client_id)
    _add_board_line((x0 + 3 * c, y0 + 7 * c, z), (x0 + 5 * c, y0 + 9 * c, z), client_id)
    _add_board_line((x0 + 5 * c, y0 + 7 * c, z), (x0 + 3 * c, y0 + 9 * c, z), client_id)


def _draw_position_marks(config: Config, client_id: int, z: float) -> None:
    mark_cells = [(1, 2), (7, 2), (1, 7), (7, 7)]
    mark_cells.extend((col, 3) for col in (0, 2, 4, 6, 8))
    mark_cells.extend((col, 6) for col in (0, 2, 4, 6, 8))
    for col, row in mark_cells:
        _draw_cross_mark(col, row, config, client_id, z)


def _draw_cross_mark(col: int, row: int, config: Config, client_id: int, z: float) -> None:
    x0, y0, _ = config.board_origin
    x = x0 + col * config.cell_size
    y = y0 + row * config.cell_size
    gap = config.cell_size * 0.12
    length = config.cell_size * 0.26
    corners = [
        (-1, -1),
        (-1, 1),
        (1, -1),
        (1, 1),
    ]
    for sx, sy in corners:
        if not 0 <= col + sx <= config.board_cols - 1:
            continue
        _add_board_line((x + sx * gap, y + sy * gap, z), (x + sx * (gap + length), y + sy * gap, z), client_id, width=1.2)
        _add_board_line((x + sx * gap, y + sy * gap, z), (x + sx * gap, y + sy * (gap + length), z), client_id, width=1.2)


def _add_board_text(
    text: str,
    position_xyz: tuple[float, float, float],
    text_size: float,
    client_id: int,
) -> None:
    debug_id = p.addUserDebugText(
        text,
        position_xyz,
        textColorRGB=_BOARD_TEXT_COLOR,
        textSize=text_size,
        physicsClientId=client_id,
    )
    RUNTIME.debug_item_ids.append(debug_id)


def _create_river_labels(config: Config, client_id: int, z: float) -> None:
    try:
        chuhe_texture = _make_label_texture("\u695a\u6cb3", "chuhe")
        hanjie_texture = _make_label_texture("\u6f22\u754c", "hanjie")
    except Exception:
        x0, y0, _ = config.board_origin
        _add_board_text("\u695a\u6cb3", (x0 + 1.6 * config.cell_size, y0 + 4.42 * config.cell_size, z), 0.9, client_id)
        _add_board_text("\u6f22\u754c", (x0 + 5.6 * config.cell_size, y0 + 4.42 * config.cell_size, z), 0.9, client_id)
        return

    x0, y0, _ = config.board_origin
    river_y = y0 + 4.5 * config.cell_size
    label_width = 2.0 * config.cell_size
    label_height = 0.72 * config.cell_size
    _create_label_plate(
        chuhe_texture,
        (x0 + 2.0 * config.cell_size, river_y, z),
        label_width,
        label_height,
        client_id,
        yaw=0.0,
    )
    _create_label_plate(
        hanjie_texture,
        (x0 + 6.0 * config.cell_size, river_y, z),
        label_width,
        label_height,
        client_id,
        yaw=math.pi,
    )


def _make_label_texture(text: str, name: str) -> Path:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.font_manager import FontProperties

    texture_dir = Path(tempfile.gettempdir()) / "chess_robot_sim_textures"
    texture_dir.mkdir(parents=True, exist_ok=True)
    texture_path = texture_dir / f"{name}.png"

    font_path = _find_chinese_font()
    font = FontProperties(fname=str(font_path), size=64) if font_path else None
    background = (0.93, 0.88, 0.74, 1.0)
    text_color = (0.24, 0.11, 0.03, 1.0)

    fig = plt.figure(figsize=(3.0, 1.0), dpi=120, facecolor=background)
    ax = fig.add_axes((0, 0, 1, 1))
    ax.set_axis_off()
    ax.set_facecolor(background)
    ax.text(0.5, 0.5, text, ha="center", va="center", color=text_color, fontproperties=font)
    fig.savefig(texture_path, dpi=120, facecolor=background)
    plt.close(fig)
    return texture_path


def _make_piece_label_texture(text: str, color: str, piece_id: str) -> Path:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib import patheffects
    from matplotlib.patches import Circle
    from matplotlib.font_manager import FontProperties

    texture_dir = Path(tempfile.gettempdir()) / "chess_robot_sim_textures" / "pieces"
    texture_dir.mkdir(parents=True, exist_ok=True)
    texture_path = texture_dir / f"{piece_id}_v2.png"

    font_path = _find_chinese_font()
    font = FontProperties(fname=str(font_path), size=72) if font_path else FontProperties(size=72)
    text_color = _RED_TEXT_COLOR if color == "red" else _BLACK_TEXT_COLOR
    ring_color = (0.52, 0.25, 0.08, 1.0)
    shadow_color = (0.18, 0.09, 0.03, 0.42)
    base_color = (0.90, 0.69, 0.39, 1.0)
    light_color = (0.98, 0.82, 0.52, 1.0)

    fig = plt.figure(figsize=(1.0, 1.0), dpi=512, facecolor=(0.0, 0.0, 0.0, 0.0))
    ax = fig.add_axes((0, 0, 1, 1))
    ax.set_axis_off()
    ax.set_xlim(0.0, 1.0)
    ax.set_ylim(0.0, 1.0)
    ax.set_aspect("equal")

    ax.add_patch(Circle((0.5, 0.5), 0.5, facecolor=base_color, edgecolor="none"))
    for index in range(18):
        radius = 0.49 - index * 0.018
        if radius <= 0.0:
            break
        alpha = 0.09 if index % 2 == 0 else 0.045
        ax.add_patch(Circle((0.5, 0.5), radius, fill=False, edgecolor=(0.72, 0.42, 0.17, alpha), linewidth=1.2))
    ax.add_patch(Circle((0.43, 0.60), 0.45, facecolor=light_color, edgecolor="none", alpha=0.18))
    ax.add_patch(Circle((0.5, 0.5), 0.485, fill=False, edgecolor=shadow_color, linewidth=8.0))
    ax.add_patch(Circle((0.5, 0.5), 0.405, fill=False, edgecolor=ring_color, linewidth=9.0))
    ax.add_patch(Circle((0.5, 0.5), 0.382, fill=False, edgecolor=ring_color, linewidth=3.7, alpha=0.95))
    ax.add_patch(Circle((0.5, 0.5), 0.455, fill=False, edgecolor=(1.0, 0.90, 0.62, 0.35), linewidth=2.0))

    label = ax.text(0.5, 0.51, text, ha="center", va="center", color=text_color, fontproperties=font)
    fig.canvas.draw()
    renderer = fig.canvas.get_renderer()
    max_extent = fig.bbox.width * 0.65
    while font.get_size_in_points() > 28:
        bbox = label.get_window_extent(renderer=renderer)
        if max(bbox.width, bbox.height) <= max_extent:
            break
        font.set_size(font.get_size_in_points() - 2)
        label.set_fontproperties(font)
        fig.canvas.draw()
    label.set_path_effects(
        [
            patheffects.withStroke(linewidth=4.0, foreground=(1.0, 0.78, 0.46, 0.72)),
            patheffects.Normal(),
        ]
    )
    fig.savefig(texture_path, dpi=512, transparent=True, pad_inches=0)
    plt.close(fig)
    return texture_path


def _find_chinese_font() -> Path | None:
    candidates = [
        Path("C:/Windows/Fonts/simkai.ttf"),
        Path("C:/Windows/Fonts/simhei.ttf"),
        Path("C:/Windows/Fonts/msyh.ttc"),
        Path("C:/Windows/Fonts/simsun.ttc"),
    ]
    for path in candidates:
        if path.exists():
            return path
    return None


def _create_label_plate(
    texture_path: Path,
    center_xyz: tuple[float, float, float],
    width: float,
    height: float,
    client_id: int,
    yaw: float = 0.0,
) -> int:
    mesh_path = project_root() / "assets" / "board" / "label_quad.obj"
    visual_id = p.createVisualShape(
        p.GEOM_MESH,
        fileName=str(mesh_path),
        meshScale=(width, height, 1.0),
        rgbaColor=(1.0, 1.0, 1.0, 1.0),
        physicsClientId=client_id,
    )
    body_id = p.createMultiBody(
        baseMass=0.0,
        baseCollisionShapeIndex=-1,
        baseVisualShapeIndex=visual_id,
        basePosition=center_xyz,
        baseOrientation=p.getQuaternionFromEuler((0.0, 0.0, yaw)),
        physicsClientId=client_id,
    )
    texture_id = p.loadTexture(str(texture_path), physicsClientId=client_id)
    p.changeVisualShape(body_id, -1, textureUniqueId=texture_id, physicsClientId=client_id)
    RUNTIME.scene_body_ids.append(body_id)
    return body_id


def _create_captured_area(config: Config, client_id: int) -> int:
    x0, y0, _ = config.board_origin
    center = (
        x0 + (config.board_cols + 1) * config.cell_size,
        y0 + (config.board_rows - 1) * config.cell_size / 2.0,
        config.z_board + 0.001,
    )
    visual_id = p.createVisualShape(
        p.GEOM_BOX,
        halfExtents=(0.025, config.board_rows * config.cell_size / 2.0, 0.002),
        rgbaColor=(0.85, 0.85, 0.90, 0.75),
        physicsClientId=client_id,
    )
    body_id = p.createMultiBody(
        baseMass=0.0,
        baseCollisionShapeIndex=-1,
        baseVisualShapeIndex=visual_id,
        basePosition=center,
        physicsClientId=client_id,
    )
    RUNTIME.scene_body_ids.append(body_id)
    _add_board_text("\u5403\u5b50\u533a", (center[0] - 0.018, center[1] - 0.08, config.z_board + 0.012), 0.55, client_id)
    return body_id


def _create_piece(
    cell: str,
    piece_id: str,
    color: str,
    label: str,
    config: Config,
    client_id: int,
) -> int:
    x, y, z = cell_to_world(cell, config)
    visual_id = p.createVisualShape(
        p.GEOM_CYLINDER,
        radius=config.piece_radius,
        length=config.piece_height,
        rgbaColor=_PIECE_WOOD_COLOR,
        physicsClientId=client_id,
    )
    collision_id = p.createCollisionShape(
        p.GEOM_CYLINDER,
        radius=config.piece_radius,
        height=config.piece_height,
        physicsClientId=client_id,
    )
    body_id = p.createMultiBody(
        baseMass=0.02,
        baseCollisionShapeIndex=collision_id,
        baseVisualShapeIndex=visual_id,
        basePosition=(x, y, z + config.piece_height / 2.0),
        physicsClientId=client_id,
    )
    p.changeDynamics(body_id, -1, lateralFriction=0.9, physicsClientId=client_id)
    p.changeVisualShape(
        body_id,
        -1,
        rgbaColor=_PIECE_WOOD_COLOR,
        specularColor=(0.28, 0.18, 0.08),
        physicsClientId=client_id,
    )
    # 文字标签盘（动态质量 0.001，确保吸附/释放时能跟随主 body 运动）
    # 不使用装饰环和顶盖，简化为圆柱 + 文字贴图两层结构
    label_id, label_constraint_id = _create_piece_label(body_id, piece_id, label, color, (x, y, z), config, client_id)
    RUNTIME.scene_body_ids.append(body_id)
    if label_id is not None:
        RUNTIME.scene_body_ids.append(label_id)
    if label_constraint_id is not None:
        RUNTIME.attachment_constraints[f"{piece_id}_label"] = label_constraint_id
    RUNTIME.piece_body_ids[piece_id] = body_id
    RUNTIME.piece_cells[piece_id] = cell
    RUNTIME.piece_ids_by_cell[cell] = piece_id
    return body_id


def _create_piece_label(
    piece_body_id: int,
    piece_id: str,
    label: str,
    color: str,
    piece_world_xyz: tuple[float, float, float],
    config: Config,
    client_id: int,
) -> tuple[int | None, int | None]:
    try:
        texture_path = _make_piece_label_texture(label, color, piece_id)
    except Exception:
        return None, None

    mesh_path = project_root() / "assets" / "board" / "piece_label_disc.obj"
    side = config.piece_radius * 1.62
    x, y, z = piece_world_xyz
    visual_id = p.createVisualShape(
        p.GEOM_MESH,
        fileName=str(mesh_path),
        meshScale=(side, side, 1.0),
        rgbaColor=(1.0, 1.0, 1.0, 1.0),
        physicsClientId=client_id,
    )
    label_body_id = p.createMultiBody(
        baseMass=0.001,
        baseCollisionShapeIndex=-1,
        baseVisualShapeIndex=visual_id,
        basePosition=(x, y, z + config.piece_height + 0.004),
        baseOrientation=p.getQuaternionFromEuler((0.0, 0.0, math.pi if color == "black" else 0.0)),
        physicsClientId=client_id,
    )
    texture_id = p.loadTexture(str(texture_path), physicsClientId=client_id)
    p.changeVisualShape(
        label_body_id,
        -1,
        textureUniqueId=texture_id,
        specularColor=(0.25, 0.18, 0.10),
        physicsClientId=client_id,
    )
    constraint_id = p.createConstraint(
        parentBodyUniqueId=piece_body_id,
        parentLinkIndex=-1,
        childBodyUniqueId=label_body_id,
        childLinkIndex=-1,
        jointType=p.JOINT_FIXED,
        jointAxis=(0.0, 0.0, 0.0),
        parentFramePosition=(0.0, 0.0, config.piece_height / 2.0 + 0.004),
        childFramePosition=(0.0, 0.0, 0.0),
        childFrameOrientation=p.getQuaternionFromEuler((0.0, 0.0, math.pi if color == "black" else 0.0)),
        physicsClientId=client_id,
    )
    return label_body_id, constraint_id


def _create_obstacle_body(obstacle: Obstacle, client_id: int) -> int:
    x, y, z = obstacle.center_xyz
    visual_id = p.createVisualShape(
        p.GEOM_CYLINDER,
        radius=obstacle.radius,
        length=obstacle.height,
        rgbaColor=(0.1, 0.25, 0.95, 0.8),
        physicsClientId=client_id,
    )
    collision_id = p.createCollisionShape(
        p.GEOM_CYLINDER,
        radius=obstacle.radius,
        height=obstacle.height,
        physicsClientId=client_id,
    )
    body_id = p.createMultiBody(
        baseMass=0.0,
        baseCollisionShapeIndex=collision_id,
        baseVisualShapeIndex=visual_id,
        basePosition=(x, y, z + obstacle.height / 2.0),
        physicsClientId=client_id,
    )
    RUNTIME.scene_body_ids.append(body_id)
    return body_id


def _set_camera(config: Config, client_id: int) -> None:
    x0, y0, _ = config.board_origin
    target = (
        x0 + (config.board_cols - 1) * config.cell_size / 2.0,
        y0 + (config.board_rows - 1) * config.cell_size / 2.0,
        config.z_board,
    )
    p.resetDebugVisualizerCamera(
        cameraDistance=0.75,
        cameraYaw=45.0,
        cameraPitch=-45.0,
        cameraTargetPosition=target,
        physicsClientId=client_id,
    )


def build_obstacle_preset(obstacle_mode: str, config: Config = DEFAULT_CONFIG) -> list[Obstacle]:
    """Return preset vertical cylinder obstacles for avoidance demos."""
    presets = {
        "mode_1": [(4, 5)],
        "mode_2": [(2, 5), (5, 5)],
        "mode_3": [(2, 5), (4, 6), (6, 5)],
        "none": [],
    }
    radii = {
        "mode_1": 0.05,
        "mode_2": 0.025,
        "mode_3": 0.045,
        "none": 0.0,
    }
    cells = presets.get(obstacle_mode)
    if cells is None:
        raise ValueError(f"unknown obstacle_mode: {obstacle_mode}")
    radius = radii[obstacle_mode]
    return [
        Obstacle(
            obstacle_id=f"preset_column_{index + 1}",
            center_xyz=(
                config.board_origin[0] + col * config.cell_size,
                config.board_origin[1] + row * config.cell_size,
                config.z_board,
            ),
            radius=radius,
            height=0.30,
            dynamic=False,
        )
        for index, (col, row) in enumerate(cells)
    ]
