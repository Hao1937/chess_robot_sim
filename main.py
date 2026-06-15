from __future__ import annotations

import argparse
from dataclasses import replace
from typing import Callable

from src.common.config import DEFAULT_CONFIG, Config
from src.common.types import BoardState, ExecutionResult, JointTrajectory, LogicalAction, MoveCommand, PieceColor, RobotHandle, SceneHandle
from src.control.controller import execute_trajectory
from src.control.logger import summarize_execution
from src.interaction.board_state import create_initial_board, make_logical_actions
from src.interaction.chess_rules import validate_move
from src.interaction.cli import parse_command
from src.interaction.gui import poll_gui_command
from src.planning.motion_primitives import build_motion_primitives, get_action_primitive_ranges
from src.planning.obstacle_map import build_primitive_obstacle_contexts
from src.planning.trajectory_planner import plan_trajectory
from src.simulation.attachment import attach_piece, detach_piece
from src.simulation.load_robot import load_robot
from src.simulation.scene_builder import build_scene, set_human_safety_zone


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
    planning_contexts = build_primitive_obstacle_contexts(
        actions=actions,
        primitives=primitives,
        board=board,
        extra_obstacles=scene.obstacles,
        human_hand_present=human_hand_present,
        config=config,
    )
    safety_decisions = [context.safety_decision for context in planning_contexts]
    obstacle_ids = sorted({
        obstacle.obstacle_id
        for context in planning_contexts
        for obstacle in context.obstacles
    })
    trajectory = plan_trajectory(planning_contexts, config=config)

    # ── 可行性验证：在 attach 任何棋子之前检查轨迹是否安全可达 ──
    from src.planning.feasibility import validate_trajectory_feasibility
    ok, reason = validate_trajectory_feasibility(planning_contexts, trajectory, config)
    if not ok:
        raise ValueError(f"路线不可达，请移除障碍物：{reason}")

    # 按 action 分段执行，在 pick/place 对之间切换 attach/detach 目标。
    # 这样吃子时敌方棋子也能正确吸附跟随、到达 captured area 后释放，
    # 然后机械臂再去抓取己方棋子。
    #
    # 关键设计：attach 必须在 EE 到达棋子位置后（descend 完成后）调用，
    # 这样棋子不需要瞬移，子部件（装饰环、顶盖）通过约束保持跟随。
    # 同样，detach 在 retreat 前调用，确保棋子先释放再移开。
    action_prim_ranges = get_action_primitive_ranges(actions)
    primitive_ranges = trajectory.primitive_ranges
    accumulated: list[ExecutionResult] = []
    attached_piece_id: str = ""
    for i, action in enumerate(actions):
        prim_start, prim_end = action_prim_ranges[i]

        if not primitive_ranges or prim_start >= len(primitive_ranges):
            # 向后兼容：primitive_ranges 不可用时回退到旧行为
            wp_start, wp_end = prim_start, prim_end
            if action.action_type == "pick" and action.piece_id:
                attached_piece_id = action.piece_id
                attach_piece(piece_id=action.piece_id, end_effector_id=robot.end_effector_id)
            segment_result = execute_trajectory(
                JointTrajectory(
                    joint_waypoints=trajectory.joint_waypoints[wp_start:wp_end],
                    speed_profile=trajectory.speed_profile[wp_start:wp_end],
                )
            )
            accumulated.append(segment_result)
            if action.action_type == "place" and attached_piece_id:
                detach_piece(piece_id=attached_piece_id)
                attached_piece_id = ""
            continue

        if action.action_type == "pick" and action.piece_id:
            # ── pick：先 approach+descend 到达棋子，再 attach，最后 grasp+lift ──
            # 前段：approach + descend (primitives prim_start .. prim_start+1)
            pre_end = prim_start + 2  # approach=0, descend=1 → end=2
            wp_pre_start = primitive_ranges[prim_start][0]
            wp_pre_end = primitive_ranges[pre_end - 1][1]
            pre_segment = JointTrajectory(
                joint_waypoints=trajectory.joint_waypoints[wp_pre_start:wp_pre_end],
                speed_profile=trajectory.speed_profile[wp_pre_start:wp_pre_end],
            )
            accumulated.append(execute_trajectory(pre_segment))

            # 在 EE 已到达棋子位置时吸附
            attached_piece_id = action.piece_id
            attach_piece(piece_id=action.piece_id, end_effector_id=robot.end_effector_id)

            # 后段：grasp + lift (primitives prim_start+2 .. prim_start+3)
            if prim_end > pre_end:
                wp_post_start = primitive_ranges[pre_end][0]
                wp_post_end = primitive_ranges[prim_end - 1][1]
                post_segment = JointTrajectory(
                    joint_waypoints=trajectory.joint_waypoints[wp_post_start:wp_post_end],
                    speed_profile=trajectory.speed_profile[wp_post_start:wp_post_end],
                )
                accumulated.append(execute_trajectory(post_segment))

        elif action.action_type == "place" and attached_piece_id:
            # ── place：先 transfer+descend 到达目标，再 detach，最后 retreat ──
            pre_end = prim_start + 2  # transfer=0, descend=1 → end=2
            wp_pre_start = primitive_ranges[prim_start][0]
            wp_pre_end = primitive_ranges[pre_end - 1][1]
            pre_segment = JointTrajectory(
                joint_waypoints=trajectory.joint_waypoints[wp_pre_start:wp_pre_end],
                speed_profile=trajectory.speed_profile[wp_pre_start:wp_pre_end],
            )
            accumulated.append(execute_trajectory(pre_segment))

            # 在 retreat 前释放棋子
            detach_piece(piece_id=attached_piece_id)
            attached_piece_id = ""

            # 后段：detach + retreat (primitives prim_start+2 .. prim_start+3)
            if prim_end > pre_end:
                wp_post_start = primitive_ranges[pre_end][0]
                wp_post_end = primitive_ranges[prim_end - 1][1]
                post_segment = JointTrajectory(
                    joint_waypoints=trajectory.joint_waypoints[wp_post_start:wp_post_end],
                    speed_profile=trajectory.speed_profile[wp_post_start:wp_post_end],
                )
                accumulated.append(execute_trajectory(post_segment))

        else:
            # ── safety_pause 等无需 attach/detach 的动作 ──
            wp_start = primitive_ranges[prim_start][0]
            wp_end = primitive_ranges[prim_end - 1][1] if prim_end > prim_start else wp_start
            segment = JointTrajectory(
                joint_waypoints=trajectory.joint_waypoints[wp_start:wp_end],
                speed_profile=trajectory.speed_profile[wp_start:wp_end],
            )
            accumulated.append(execute_trajectory(segment))

    execution = _merge_executions(accumulated)

    if execution.success:
        apply_logical_actions(board, actions)

    summary = summarize_execution(execution)
    return {
        "command": command,
        "actions": actions,
        "human_hand_present": human_hand_present,
        "obstacle_ids": obstacle_ids,
        "planning_contexts": planning_contexts,
        "safety_decisions": safety_decisions,
        "primitive_count": len(primitives),
        "trajectory_points": len(trajectory.joint_waypoints),
        "execution": execution,
        "summary": summary,
    }


def run_demo(command_text: str = "A1 A2") -> dict[str, object]:
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
            set_human_safety_zone(True, config)
        elif command.command_type == "hand_off":
            human_hand_present = False
            set_human_safety_zone(False, config)

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


def _merge_executions(segments: list[ExecutionResult]) -> ExecutionResult:
    """将多个分段执行结果合并为一个 ExecutionResult。"""
    if not segments:
        return ExecutionResult(
            success=True,
            desired_joint_angles=[],
            actual_joint_angles=[],
            joint_errors=[],
            end_effector_errors=[],
            obstacle_clearances=[],
            execution_time=0.0,
        )
    return ExecutionResult(
        success=all(segment.success for segment in segments),
        desired_joint_angles=[wp for segment in segments for wp in segment.desired_joint_angles],
        actual_joint_angles=[wp for segment in segments for wp in segment.actual_joint_angles],
        joint_errors=[e for segment in segments for e in segment.joint_errors],
        end_effector_errors=[e for segment in segments for e in segment.end_effector_errors],
        obstacle_clearances=[c for segment in segments for c in segment.obstacle_clearances],
        execution_time=round(sum(segment.execution_time for segment in segments), 3),
    )


def main() -> None:
    parser = argparse.ArgumentParser(description="Chinese chess robot arm simulation skeleton")
    parser.add_argument("--demo", action="store_true", help="run one mock command and exit")
    parser.add_argument("--interactive", action="store_true", help="keep the session open and poll commands")
    parser.add_argument("--command", default="A1 A2", help="move command, for example: A1 A2")
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
        _maybe_wait_gui()
    else:
        parser.print_help()


def _maybe_wait_gui() -> None:
    """GUI 模式下保持窗口打开，直到用户按 Enter。"""
    import os
    if os.environ.get("CHESS_ROBOT_PYBULLET_GUI") != "1":
        return
    try:
        from src.simulation._runtime import RUNTIME, p
        if p is not None and RUNTIME.client_id is not None:
            print("\n[GUI] PyBullet 窗口保持打开，按 Enter 关闭...")
            try:
                input()
            except EOFError:
                pass
            try:
                p.disconnect(RUNTIME.client_id)
            except Exception:
                pass
    except Exception:
        pass


if __name__ == "__main__":
    main()
