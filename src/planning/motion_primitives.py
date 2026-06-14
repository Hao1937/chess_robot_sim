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
        #elif action.action_type == ''
        else:
            print('Error, primitives not defined!')
    return primitives
