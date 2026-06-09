from __future__ import annotations

import argparse

from src.common.config import DEFAULT_CONFIG
from src.control.controller import execute_trajectory
from src.control.logger import summarize_execution
from src.interaction.board_state import create_initial_board, make_logical_actions
from src.interaction.chess_rules import validate_move
from src.interaction.cli import parse_command
from src.planning.motion_primitives import build_motion_primitives
from src.planning.obstacle_map import build_obstacle_map
from src.planning.trajectory_planner import plan_trajectory
from src.simulation.attachment import attach_piece, detach_piece
from src.simulation.load_robot import load_robot
from src.simulation.scene_builder import build_scene


def run_demo(command_text: str = "A1 B1") -> dict[str, object]:
    """Run a mock end-to-end pipeline through A/B/C/D interfaces."""
    config = DEFAULT_CONFIG
    command = parse_command(command_text)
    obstacle_mode = command.mode if command.command_type == "obstacle_mode" else "mode_1"
    robot = load_robot()
    scene = build_scene(config=config, obstacle_mode=obstacle_mode)
    board = create_initial_board()

    validation = validate_move(board, command)
    if not validation.is_legal:
        raise ValueError(validation.reason)

    actions = make_logical_actions(board, command)
    primitives = build_motion_primitives(actions, config)
    obstacles = build_obstacle_map(piece_cells=list(board.pieces), extra_obstacles=scene.obstacles, config=config)
    trajectory = plan_trajectory(primitives, obstacles, config)

    should_attach_piece = command.command_type == "move" and command.from_cell in board.pieces
    if should_attach_piece:
        attach_piece(piece_id=board.pieces[command.from_cell].piece_id, end_effector_id=robot.end_effector_id)
    execution = execute_trajectory(trajectory)
    if should_attach_piece:
        detach_piece(piece_id=board.pieces[command.from_cell].piece_id)

    summary = summarize_execution(execution)
    return {
        "command": command,
        "actions": actions,
        "primitive_count": len(primitives),
        "trajectory_points": len(trajectory.joint_waypoints),
        "execution": execution,
        "summary": summary,
    }


def main() -> None:
    parser = argparse.ArgumentParser(description="Chinese chess robot arm simulation skeleton")
    parser.add_argument("--demo", action="store_true", help="run mock A1 B1 demo")
    parser.add_argument("--command", default="A1 B1", help="move command, for example: A1 B1")
    args = parser.parse_args()

    if args.demo:
        result = run_demo(args.command)
        print("Demo command:", result["command"])
        print("Logical actions:", result["actions"])
        print("Motion primitives:", result["primitive_count"])
        print("Trajectory points:", result["trajectory_points"])
        print("Execution summary:", result["summary"])
    else:
        parser.print_help()


if __name__ == "__main__":
    main()
