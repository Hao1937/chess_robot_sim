from __future__ import annotations

from src.common.config import DEFAULT_CONFIG, Config
from src.common.types import LogicalAction, MotionPrimitive
from src.planning.chessboard_mapping import cell_above_world, cell_to_world


def build_motion_primitives(actions: list[LogicalAction], config: Config = DEFAULT_CONFIG) -> list[MotionPrimitive]:
    """Convert logical actions into robot motion primitives."""
    primitives: list[MotionPrimitive] = []
    for action in actions:
        if action.action_type == "pick":
            world = cell_to_world(action.cell, config)
            primitives.append(MotionPrimitive("approach", action.cell, cell_above_world(action.cell, config), "fast", action.piece_id))
            primitives.append(MotionPrimitive("descend", action.cell, (world[0], world[1], world[2] + config.z_grasp), "safe", action.piece_id))
            primitives.append(MotionPrimitive("grasp", action.cell, (world[0], world[1], world[2] + config.z_grasp), "safe", action.piece_id))
            primitives.append(MotionPrimitive("lift", action.cell, cell_above_world(action.cell, config), "safe", action.piece_id))
        elif action.action_type == "place":
            world = cell_to_world(action.cell, config)
            primitives.append(MotionPrimitive("transfer", action.cell, cell_above_world(action.cell, config), "fast", action.piece_id))
            primitives.append(MotionPrimitive("descend", action.cell, (world[0], world[1], world[2] + config.z_grasp), "safe", action.piece_id))
            primitives.append(MotionPrimitive("detach", action.cell, (world[0], world[1], world[2] + config.z_grasp), "safe", action.piece_id))
            primitives.append(MotionPrimitive("retreat", action.cell, cell_above_world(action.cell, config), "safe", action.piece_id))
        elif action.action_type == "safety_pause":
            primitives.append(MotionPrimitive("pause", action.cell, (0.0, 0.0, config.z_safe), "safe"))
        else:
            print('Error, primitives not defined!')
    return primitives


def get_action_primitive_ranges(actions: list[LogicalAction]) -> list[tuple[int, int]]:
    """返回每个 LogicalAction 对应的 motion primitive 的 [start, end) 索引区间。

    这使得 main.py 可以在每个 pick/place 对之间切换 attach/detach 目标，
    解决吃子时多棋子搬运的吸附覆盖问题。
    """
    ranges: list[tuple[int, int]] = []
    offset = 0
    for action in actions:
        if action.action_type in ("pick", "place"):
            count = 4  # approach, descend, grasp/lift 或 transfer, descend, detach, retreat
        elif action.action_type == "safety_pause":
            count = 1
        else:
            count = 1
        ranges.append((offset, offset + count))
        offset += count
    return ranges
