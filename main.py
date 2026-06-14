from __future__ import annotations

import argparse
from dataclasses import replace
from typing import Callable

from src.common.config import DEFAULT_CONFIG, Config
from src.common.types import BoardState, LogicalAction, MoveCommand, PieceColor, RobotHandle, SceneHandle
from src.control.controller import execute_trajectory
from src.control.logger import summarize_execution
from src.interaction.board_state import create_initial_board, make_logical_actions
from src.interaction.chess_rules import validate_move
from src.interaction.cli import parse_command
from src.interaction.gui import poll_gui_command
from src.planning.motion_primitives import build_motion_primitives
from src.planning.obstacle_map import assess_obstacle_intervention, build_obstacle_map
from src.planning.trajectory_planner import plan_trajectory
from src.simulation.attachment import attach_piece, detach_piece
from src.simulation.load_robot import load_robot
from src.simulation.scene_builder import build_scene


InputFunc = Callable[[str], str]
OutputFunc = Callable[[str], None]


def run_command(
    command: MoveCommand,
    board: BoardState,
    scene: SceneHandle,
    robot: RobotHandle,
    config: Config = DEFAULT_CONFIG,
    *,
    human_hand_present: bool = False,
) -> dict[str, object]:
    """Run one command through the A/B/C/D pipeline using an existing session."""
    validation = validate_move(board, command)
    if not validation.is_legal:
        raise ValueError(validation.reason)

    actions = make_logical_actions(board, command)
    primitives = build_motion_primitives(actions, config)
    obstacles = build_obstacle_map(
        piece_cells=list(board.pieces),
        extra_obstacles=scene.obstacles,
        human_hand_present=human_hand_present,
        config=config,
    )
    safety_decisions = [
        assess_obstacle_intervention(primitive.target_xyz, obstacles, config)
        for primitive in primitives
    ]
    trajectory = plan_trajectory(primitives, obstacles, config)

    should_attach_piece = command.command_type == "move" and command.from_cell in board.pieces
    if should_attach_piece:
        attach_piece(piece_id=board.pieces[command.from_cell].piece_id, end_effector_id=robot.end_effector_id)
    execution = execute_trajectory(trajectory)
    if should_attach_piece:
        detach_piece(piece_id=board.pieces[command.from_cell].piece_id)

    if execution.success:
        apply_logical_actions(board, actions)

    summary = summarize_execution(execution)
    return {
        "command": command,
        "actions": actions,
        "human_hand_present": human_hand_present,
        "obstacle_ids": [obstacle.obstacle_id for obstacle in obstacles],
        "safety_decisions": safety_decisions,
        "primitive_count": len(primitives),
        "trajectory_points": len(trajectory.joint_waypoints),
        "execution": execution,
        "summary": summary,
    }


def run_demo(command_text: str = "A1 B1") -> dict[str, object]:
    """Run a mock one-shot pipeline through A/B/C/D interfaces."""
    config = DEFAULT_CONFIG
    command = parse_command(command_text)
    obstacle_mode = command.mode if command.command_type == "obstacle_mode" else "mode_1"
    robot = load_robot()
    scene = build_scene(config=config, obstacle_mode=obstacle_mode)
    board = create_initial_board()
    return run_command(
        command,
        board,
        scene,
        robot,
        config,
        human_hand_present=command.command_type == "hand_on",
    )


def run_interactive(
    input_func: InputFunc = input,
    output_func: OutputFunc = print,
    config: Config = DEFAULT_CONFIG,
    max_steps: int | None = None,
) -> dict[str, object]:
    """Keep robot, scene, and board alive while polling commands continuously."""
    board = create_initial_board()
    robot = load_robot()
    scene = build_scene(config=config, obstacle_mode="mode_1")
    results: list[dict[str, object]] = []
    human_hand_present = False
    steps = 0

    output_func("interactive session started; enter moves, obstacle_mode N, reset, hand_on/off, or quit")
    while True:
        command = poll_gui_command(board, input_func=input_func)
        if command is None:
            continue
        if command.command_type == "quit":
            output_func("interactive session ended")
            break

        if command.command_type == "hand_on":
            human_hand_present = True
        elif command.command_type == "hand_off":
            human_hand_present = False

        if command.command_type == "obstacle_mode":
            scene = build_scene(config=config, obstacle_mode=command.mode)

        try:
            result = run_command(
                command,
                board,
                scene,
                robot,
                config,
                human_hand_present=human_hand_present,
            )
        except ValueError as exc:
            output_func(f"error: {exc}")
            continue

        results.append(result)
        output_func(f"ok: {command.command_type}")
        steps += 1
        if max_steps is not None and steps >= max_steps:
            output_func("interactive session ended")
            break

    return {
        "board": board,
        "scene": scene,
        "robot": robot,
        "human_hand_present": human_hand_present,
        "results": results,
    }


def apply_logical_actions(board: BoardState, actions: list[LogicalAction]) -> None:
    """Apply successful pick/place actions to the in-memory board state."""
    carried = {}
    for action in actions:
        if action.action_type == "pick":
            cell = _find_piece_cell(board, action.piece_id) if action.piece_id else action.cell
            piece = board.pieces.pop(cell)
            carried[piece.piece_id] = piece
        elif action.action_type == "place":
            piece_id = action.piece_id or next(iter(carried))
            piece = carried.pop(piece_id)
            board.pieces[action.cell] = replace(piece, cell=action.cell)
    _refresh_captured_counts(board)


def _find_piece_cell(board: BoardState, piece_id: str) -> str:
    for cell, piece in board.pieces.items():
        if piece.piece_id == piece_id:
            return cell
    raise ValueError(f"piece not found: {piece_id}")


def _refresh_captured_counts(board: BoardState) -> None:
    counts = {PieceColor.RED: 0, PieceColor.BLACK: 0}
    for cell, piece in board.pieces.items():
        if cell.startswith("CAPTURED_"):
            counts[piece.color] += 1
    board.captured_counts = counts


def main() -> None:
    parser = argparse.ArgumentParser(description="Chinese chess robot arm simulation skeleton")
    parser.add_argument("--demo", action="store_true", help="run one mock command and exit")
    parser.add_argument("--interactive", action="store_true", help="keep the session open and poll commands")
    parser.add_argument("--command", default="A1 B1", help="move command, for example: A1 B1")
    args = parser.parse_args()

    if args.interactive:
        run_interactive()
    elif args.demo:
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
