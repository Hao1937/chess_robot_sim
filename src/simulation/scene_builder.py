from __future__ import annotations

from src.common.config import DEFAULT_CONFIG, Config
from src.common.types import Obstacle, SceneHandle
from src.planning.chessboard_mapping import cell_to_world
from src.simulation._runtime import RUNTIME, clear_scene_bodies, ensure_client, p


_DEMO_PIECES = (
    ("A1", "red_rook_1", "red", "\u8f66"),
    ("B1", "black_horse_1", "black", "\u9a6c"),
    ("C1", "red_cannon_1", "red", "\u70ae"),
)


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

    clear_scene_bodies()
    _create_table(config, client_id)
    board_id = _create_board(config, client_id)
    _draw_board_grid(config, client_id)
    _create_captured_area(config, client_id)

    piece_ids: dict[str, int] = {}
    for cell, piece_id, color, label in _DEMO_PIECES:
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

    center = (
        config.board_origin[0] + 4 * config.cell_size,
        config.board_origin[1] + 4 * config.cell_size,
        config.z_board + 0.06,
    )
    visual_id = p.createVisualShape(
        p.GEOM_CYLINDER,
        radius=0.08,
        length=0.12,
        rgbaColor=(1.0, 0.0, 0.0, 0.25),
        physicsClientId=client_id,
    )
    collision_id = p.createCollisionShape(
        p.GEOM_CYLINDER,
        radius=0.08,
        height=0.12,
        physicsClientId=client_id,
    )
    body_id = p.createMultiBody(
        baseMass=0.0,
        baseCollisionShapeIndex=collision_id,
        baseVisualShapeIndex=visual_id,
        basePosition=center,
        physicsClientId=client_id,
    )
    RUNTIME.human_zone_body_id = body_id
    RUNTIME.scene_body_ids.append(body_id)
    return body_id


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


def _draw_board_grid(config: Config, client_id: int) -> None:
    x0, y0, _ = config.board_origin
    z = config.z_board + 0.001
    x_max = x0 + (config.board_cols - 1) * config.cell_size
    y_max = y0 + (config.board_rows - 1) * config.cell_size
    color = (0.05, 0.05, 0.05)
    for col in range(config.board_cols):
        x = x0 + col * config.cell_size
        debug_id = p.addUserDebugLine((x, y0, z), (x, y_max, z), color, physicsClientId=client_id)
        RUNTIME.debug_item_ids.append(debug_id)
    for row in range(config.board_rows):
        y = y0 + row * config.cell_size
        debug_id = p.addUserDebugLine((x0, y, z), (x_max, y, z), color, physicsClientId=client_id)
        RUNTIME.debug_item_ids.append(debug_id)


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
    rgba = (0.86, 0.05, 0.05, 1.0) if color == "red" else (0.02, 0.02, 0.02, 1.0)
    visual_id = p.createVisualShape(
        p.GEOM_CYLINDER,
        radius=config.piece_radius,
        length=config.piece_height,
        rgbaColor=rgba,
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
    text_id = p.addUserDebugText(
        label,
        (x - config.piece_radius / 2.0, y - config.piece_radius / 2.0, z + config.piece_height + 0.004),
        textColorRGB=(1.0, 1.0, 1.0) if color == "black" else (0.6, 0.0, 0.0),
        textSize=0.7,
        physicsClientId=client_id,
    )
    RUNTIME.scene_body_ids.append(body_id)
    RUNTIME.debug_item_ids.append(text_id)
    RUNTIME.piece_body_ids[piece_id] = body_id
    RUNTIME.piece_cells[piece_id] = cell
    RUNTIME.piece_ids_by_cell[cell] = piece_id
    return body_id


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
    """Return 2-3 preset vertical cylinder obstacles for avoidance demos."""
    presets = {
        "mode_1": [(2, 1), (4, 1)],
        "mode_2": [(3, 2), (4, 4), (6, 3)],
        "mode_3": [(1, 3), (5, 5), (7, 2)],
        "none": [],
    }
    cells = presets.get(obstacle_mode)
    if cells is None:
        raise ValueError(f"unknown obstacle_mode: {obstacle_mode}")
    return [
        Obstacle(
            obstacle_id=f"preset_column_{index + 1}",
            center_xyz=(
                config.board_origin[0] + col * config.cell_size,
                config.board_origin[1] + row * config.cell_size,
                config.z_board,
            ),
            radius=config.inflated_piece_radius,
            height=0.08,
            dynamic=False,
        )
        for index, (col, row) in enumerate(cells)
    ]
