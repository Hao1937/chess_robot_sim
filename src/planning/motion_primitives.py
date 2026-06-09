from __future__ import annotations

from src.common.config import DEFAULT_CONFIG, Config
from src.common.types import LogicalAction, MotionPrimitive
from src.planning.chessboard_mapping import cell_above_world, cell_to_world


def build_motion_primitives(actions: list[LogicalAction], config: Config = DEFAULT_CONFIG) -> list[MotionPrimitive]:
    """Convert logical actions into robot motion primitives."""
    primitives: list[MotionPrimitive] = []
    for action in actions:
        if action.action_type == "pick":
            primitives.append(MotionPrimitive("approach", action.cell, cell_above_world(action.cell, config), "fast"))
            primitives.append(MotionPrimitive("descend", action.cell, cell_to_world(action.cell, config), "safe"))
            primitives.append(MotionPrimitive("lift", action.cell, cell_above_world(action.cell, config), "safe"))
        elif action.action_type == "place":
            primitives.append(MotionPrimitive("transfer", action.cell, cell_above_world(action.cell, config), "fast"))
            primitives.append(MotionPrimitive("descend", action.cell, cell_to_world(action.cell, config), "safe"))
            primitives.append(MotionPrimitive("retreat", action.cell, cell_above_world(action.cell, config), "safe"))
        elif action.action_type == "safety_pause":
            primitives.append(MotionPrimitive("pause", action.cell, (0.0, 0.0, config.z_safe), "safe"))
    return primitives
