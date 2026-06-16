import sys
import unittest
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT))


def _position_ee_over_piece(piece_id: str) -> None:
    """把机械臂末端定位到指定棋子上方（仅 PyBullet 模式；mock 模式为空操作）。

    用于让 attach_piece 的 3cm 前置距离检查通过。通过 solve_ik 求解棋子位置的
    关节角并 resetJointState 直接定位（测试场景，无需平滑轨迹）。
    """
    try:
        from src.simulation._runtime import RUNTIME, p
    except ImportError:
        return
    if p is None or RUNTIME.robot_id is None or not RUNTIME.joint_indices:
        return
    body_id = RUNTIME.piece_body_ids.get(piece_id)
    if body_id is None:
        return
    from src.common.config import DEFAULT_CONFIG
    from src.planning.ik_solver import solve_ik

    piece_pos = p.getBasePositionAndOrientation(body_id, physicsClientId=RUNTIME.client_id)[0]
    # 目标：吸盘尖端落在棋子顶面附近（z 抬高 grasp 高度）
    target = (piece_pos[0], piece_pos[1], DEFAULT_CONFIG.z_grasp)
    joints = solve_ik(target, DEFAULT_CONFIG, seed=DEFAULT_CONFIG.home_pose[:6])
    for idx, j in enumerate(RUNTIME.joint_indices[:6]):
        p.resetJointState(RUNTIME.robot_id, j, joints[idx], physicsClientId=RUNTIME.client_id)
        # 同时把电机目标设到该解，否则 load_robot 设置的 home 保持电机会把机器人拉回
        p.setJointMotorControl2(
            RUNTIME.robot_id, j, p.POSITION_CONTROL,
            targetPosition=joints[idx], force=800,
            physicsClientId=RUNTIME.client_id,
        )
    for _ in range(30):
        p.stepSimulation(RUNTIME.client_id)


def chinese_rook_move() -> str:
    return chr(0x8f66) + chr(0x4e8c) + chr(0x5e73) + chr(0x4e03)


def chinese_cannon_move() -> str:
    return chr(0x70ae) + chr(0x4e94) + chr(0x8fdb) + chr(0x56db)


def chinese_front_rook_move() -> str:
    return chr(0x524d) + chinese_rook_move()


def chinese_rook_retreat() -> str:
    return chr(0x8f66) + chr(0x4e8c) + chr(0x9000) + chr(0x4e00)


def chinese_cannon_retreat() -> str:
    return chr(0x70ae) + chr(0x4e94) + chr(0x9000) + chr(0x4e8c)


def chinese_black_rook_horizontal() -> str:
    return chr(0x8eca) + chr(0x4e00) + chr(0x5e73) + chr(0x4e5d)


def chinese_black_cannon_advance() -> str:
    return chr(0x7832) + chr(0x4e94) + chr(0x9032) + chr(0x56db)


def chinese_black_front_rook() -> str:
    return chr(0x524d) + chr(0x8eca) + chr(0x4e8c) + chr(0x5e73) + chr(0x4e03)


def chinese_horse_advance() -> str:
    return chr(0x9a6c) + chr(0x4e8c) + chr(0x8fdb) + chr(0x4e09)


class ContractInterfaceTests(unittest.TestCase):
    def test_initial_board_matches_simulation_full_board_layout(self):
        from src.interaction.board_state import create_initial_board
        from src.simulation.scene_builder import _INITIAL_PIECES

        board = create_initial_board()
        scene_layout = {cell: (piece_id, color) for cell, piece_id, color, _label in _INITIAL_PIECES}

        self.assertEqual(len(board.pieces), 32)
        self.assertEqual(set(board.pieces), set(scene_layout))
        self.assertEqual(len({piece.piece_id for piece in board.pieces.values()}), 32)
        for cell, piece in board.pieces.items():
            self.assertEqual((piece.piece_id, piece.color.value), scene_layout[cell])

    def test_interaction_produces_capture_actions(self):
        from src.common.types import Piece, PieceColor, PieceType
        from src.interaction.board_state import create_initial_board, make_logical_actions
        from src.interaction.chess_rules import validate_move
        from src.interaction.cli import parse_command

        board = create_initial_board()
        board.pieces["A1"] = Piece(piece_id="red_rook_1", kind=PieceType.ROOK, color=PieceColor.RED, cell="A1")
        board.pieces["B1"] = Piece(piece_id="black_horse_1", kind=PieceType.HORSE, color=PieceColor.BLACK, cell="B1")

        command = parse_command("A1 B1")
        result = validate_move(board, command)
        actions = make_logical_actions(board, command)

        self.assertTrue(result.is_legal, result.reason)
        self.assertEqual(command.from_cell, "A1")
        self.assertEqual(command.to_cell, "B1")
        self.assertEqual([action.action_type for action in actions], ["pick", "place", "pick", "place"])
        self.assertEqual(actions[1].cell, "CAPTURED_BLACK_1")

    def test_rook_cannot_jump_over_intervening_piece(self):
        from src.common.types import BoardState, Piece, PieceColor, PieceType
        from src.interaction.chess_rules import validate_move
        from src.interaction.cli import parse_command

        board = BoardState(pieces={
            "A1": Piece(piece_id="red_rook_1", kind=PieceType.ROOK, color=PieceColor.RED, cell="A1"),
            "A2": Piece(piece_id="black_horse_1", kind=PieceType.HORSE, color=PieceColor.BLACK, cell="A2"),
            "A4": Piece(piece_id="black_soldier_1", kind=PieceType.SOLDIER, color=PieceColor.BLACK, cell="A4"),
            "D1": Piece(piece_id="red_cannon_1", kind=PieceType.CANNON, color=PieceColor.RED, cell="D1"),
            "F1": Piece(piece_id="black_soldier_2", kind=PieceType.SOLDIER, color=PieceColor.BLACK, cell="F1"),
        })

        vertical = validate_move(board, parse_command("A1 A4"))
        horizontal = validate_move(board, parse_command("A1 F1"))

        self.assertFalse(vertical.is_legal)
        self.assertIn("blocked", vertical.reason)
        self.assertFalse(horizontal.is_legal)
        self.assertIn("blocked", horizontal.reason)

    def test_rook_jump_can_be_allowed_for_interactive2_only(self):
        from src.common.types import BoardState, Piece, PieceColor, PieceType
        from src.interaction.chess_rules import validate_move
        from src.interaction.cli import parse_command

        board = BoardState(pieces={
            "A1": Piece(piece_id="red_rook_1", kind=PieceType.ROOK, color=PieceColor.RED, cell="A1"),
            "A2": Piece(piece_id="black_horse_1", kind=PieceType.HORSE, color=PieceColor.BLACK, cell="A2"),
            "A4": Piece(piece_id="black_soldier_1", kind=PieceType.SOLDIER, color=PieceColor.BLACK, cell="A4"),
            "B1": Piece(piece_id="red_cannon_1", kind=PieceType.CANNON, color=PieceColor.RED, cell="B1"),
        })

        jump_capture = parse_command("A1 A4")
        diagonal = parse_command("A1 B2")
        friendly_target = parse_command("A1 B1")

        self.assertTrue(
            validate_move(board, jump_capture, allow_rook_jumps=True).is_legal
        )
        self.assertFalse(
            validate_move(board, diagonal, allow_rook_jumps=True).is_legal
        )
        self.assertFalse(
            validate_move(board, friendly_target, allow_rook_jumps=True).is_legal
        )

    def test_planning_builds_motion_primitives_and_speed_profile(self):
        from src.common.config import DEFAULT_CONFIG
        from src.common.types import LogicalAction
        from src.planning.chessboard_mapping import cell_to_world
        from src.planning.motion_primitives import build_motion_primitives
        from src.planning.obstacle_map import build_obstacle_map
        from src.planning.trajectory_planner import plan_trajectory

        start_xyz = cell_to_world("A1", DEFAULT_CONFIG)
        actions = [
            LogicalAction(action_type="pick", cell="A1"),
            LogicalAction(action_type="place", cell="B1"),
        ]
        obstacles = build_obstacle_map(piece_cells=["C1"], extra_obstacles=[])
        primitives = build_motion_primitives(actions, DEFAULT_CONFIG)
        trajectory = plan_trajectory(primitives, obstacles, DEFAULT_CONFIG)

        self.assertEqual(start_xyz, DEFAULT_CONFIG.board_origin)
        self.assertGreaterEqual(len(primitives), 2)
        self.assertGreater(len(trajectory.joint_waypoints), 0)
        self.assertEqual(len(trajectory.joint_waypoints), len(trajectory.speed_profile))
        self.assertIn("safe", trajectory.speed_profile)

    def test_primitive_obstacle_contexts_track_carried_piece_lifecycle(self):
        from src.interaction.board_state import create_initial_board, make_logical_actions
        from src.interaction.cli import parse_command
        from src.planning.motion_primitives import build_motion_primitives
        from src.planning.obstacle_map import build_primitive_obstacle_contexts
        from src.planning.trajectory_planner import plan_trajectory

        board = create_initial_board()
        actions = make_logical_actions(board, parse_command("A1 A2"))
        primitives = build_motion_primitives(actions)

        contexts = build_primitive_obstacle_contexts(
            actions=actions,
            primitives=primitives,
            board=board,
            extra_obstacles=[],
        )
        ids_by_type = {
            context.primitive.primitive_type: {obstacle.obstacle_id for obstacle in context.obstacles}
            for context in contexts
        }
        trajectory = plan_trajectory(contexts)

        self.assertEqual(len(contexts), len(primitives))
        self.assertIn("piece_A1", ids_by_type["approach"])
        self.assertNotIn("piece_A1", ids_by_type["lift"])
        self.assertNotIn("piece_A1", ids_by_type["transfer"])
        self.assertNotIn("piece_A2", ids_by_type["detach"])
        self.assertIn("piece_A2", ids_by_type["retreat"])
        # 插值后 waypoint 数 ≥ primitive 数（默认开启插值）
        self.assertGreaterEqual(len(trajectory.joint_waypoints), len(contexts))
        self.assertEqual(len(trajectory.joint_waypoints), len(trajectory.speed_profile))

    def test_solve_ik_uses_analytic_backend_without_pybullet(self):
        import math

        from src.common.config import DEFAULT_CONFIG
        from src.planning.ik_solver import solve_ik

        target = (
            DEFAULT_CONFIG.base_link_position[0] + 0.35,
            DEFAULT_CONFIG.base_link_position[1] + 0.05,
            DEFAULT_CONFIG.base_link_position[2] + 0.20,
        )
        solution = solve_ik(target, DEFAULT_CONFIG)

        self.assertEqual(len(solution), 6)
        self.assertTrue(all(math.isfinite(theta) for theta in solution))
        self.assertNotEqual(solution, DEFAULT_CONFIG.home_pose)

    def test_simulation_and_control_stubs_share_contract(self):
        from src.common.types import JointTrajectory
        from src.control.controller import execute_trajectory
        from src.control.logger import summarize_execution
        from src.simulation.attachment import attach_piece, detach_piece
        from src.simulation.load_robot import load_robot
        from src.simulation.scene_builder import build_scene

        robot = load_robot()
        scene = build_scene()
        trajectory = JointTrajectory(
            joint_waypoints=[(0.0, 0.1, 0.2, -0.3, 0.4, 0.5), (0.1, 0.2, 0.3, -0.4, 0.3, 0.6)],
            speed_profile=["fast", "safe"],
        )

        # attach_piece 有 3cm 前置距离检查（EE 必须在棋子正上方）。
        # 真实 PyBullet 下需先把 EE 定位到棋子位置；mock 下 _position_ee_over_piece 为空操作。
        _position_ee_over_piece("red_rook_1")
        attach_result = attach_piece(piece_id="red_rook_1", end_effector_id=robot.end_effector_id)
        detach_result = detach_piece(piece_id="red_rook_1")
        execution = execute_trajectory(trajectory)
        summary = summarize_execution(execution)

        self.assertTrue(scene.board_id)
        self.assertTrue(attach_result.success)
        self.assertTrue(detach_result.success)
        self.assertTrue(execution.success)
        self.assertIn("max_joint_error", summary)

    def test_obstacle_mode_command_and_scene_presets(self):
        from src.interaction.cli import parse_command
        from src.simulation.scene_builder import build_scene

        command = parse_command("obstacle_mode 2")
        scene = build_scene(obstacle_mode=command.mode)

        self.assertEqual(command.command_type, "obstacle_mode")
        self.assertEqual(command.mode, "mode_2")
        self.assertGreaterEqual(len(scene.obstacles), 1)
        self.assertTrue(all(obstacle.obstacle_id.startswith("preset_") for obstacle in scene.obstacles))

    def test_obstacle_presets_create_demo_gate_and_staggered_pressure_layout(self):
        from src.common.config import DEFAULT_CONFIG
        from src.simulation.scene_builder import build_obstacle_preset

        def cells_for(mode: str) -> list[tuple[int, int]]:
            cells = []
            for obstacle in build_obstacle_preset(mode, DEFAULT_CONFIG):
                x, y, _ = obstacle.center_xyz
                col = round((x - DEFAULT_CONFIG.board_origin[0]) / DEFAULT_CONFIG.cell_size)
                row = round((y - DEFAULT_CONFIG.board_origin[1]) / DEFAULT_CONFIG.cell_size)
                cells.append((col, row))
            return cells

        mode_1 = build_obstacle_preset("mode_1", DEFAULT_CONFIG)
        mode_2 = build_obstacle_preset("mode_2", DEFAULT_CONFIG)
        mode_3 = build_obstacle_preset("mode_3", DEFAULT_CONFIG)

        self.assertEqual(cells_for("mode_1"), [(2, 4)])       # C5 — 细圆柱
        self.assertEqual(cells_for("mode_2"), [(4, 4)])       # E5 — 立方体
        self.assertEqual(cells_for("mode_3"), [(1, 5), (4, 5)])  # B6+E6
        # mode_1：细圆柱 r=0.012 h=0.22 | mode_2：立方体 r=0.03 h=0.06 | mode_3：球+方 r=0.025 h=0.05
        self.assertTrue(all(obstacle.radius == 0.012 for obstacle in mode_1))
        self.assertTrue(all(obstacle.radius == 0.030 for obstacle in mode_2))
        self.assertTrue(all(obstacle.radius == 0.025 for obstacle in mode_3))
        self.assertTrue(all(obstacle.height == 0.22 for obstacle in mode_1))
        self.assertTrue(all(obstacle.height == 0.060 for obstacle in mode_2))
        self.assertTrue(all(obstacle.height == 0.050 for obstacle in mode_3))

    def test_member_a_day_one_command_validation_edges(self):
        from src.interaction.cli import parse_command

        move = parse_command("a1 b1")
        self.assertEqual(move.from_cell, "A1")
        self.assertEqual(move.to_cell, "B1")
        self.assertEqual(parse_command("RESET").command_type, "reset")
        self.assertEqual(parse_command("Hand_On").command_type, "hand_on")
        self.assertEqual(parse_command("OBSTACLE_MODE 3").mode, "mode_3")

        for bad_command in ["", "A1", "A1 Z9", "foo bar", "obstacle_mode 4"]:
            with self.subTest(command=bad_command):
                with self.assertRaises(ValueError):
                    parse_command(bad_command)

    def test_member_a_day_one_initial_board_and_friendly_target_guard(self):
        from src.interaction.board_state import create_initial_board, make_logical_actions
        from src.interaction.cli import parse_command

        board = create_initial_board()

        self.assertEqual(board.pieces["A1"].piece_id, "red_rook_1")
        self.assertEqual(board.pieces["B1"].piece_id, "red_horse_1")
        self.assertEqual(board.pieces["B3"].piece_id, "red_cannon_1")
        self.assertEqual(len({piece.piece_id for piece in board.pieces.values()}), len(board.pieces))

        with self.assertRaisesRegex(ValueError, "friendly"):
            make_logical_actions(board, parse_command("A1 B1"))

    def test_member_a_day_one_reset_actions_return_pieces_home(self):
        from src.common.types import Piece, PieceColor, PieceType
        from src.interaction.board_state import make_logical_actions
        from src.interaction.cli import parse_command
        from src.common.types import BoardState

        board = BoardState(
            pieces={
                "D4": Piece(piece_id="red_rook_1", kind=PieceType.ROOK, color=PieceColor.RED, cell="D4"),
                "B10": Piece(piece_id="black_horse_1", kind=PieceType.HORSE, color=PieceColor.BLACK, cell="B10"),
                "B3": Piece(piece_id="red_cannon_1", kind=PieceType.CANNON, color=PieceColor.RED, cell="B3"),
            }
        )

        actions = make_logical_actions(board, parse_command("reset"))

        self.assertEqual([action.action_type for action in actions], ["pick", "place"])
        self.assertEqual(actions[0].cell, "D4")
        self.assertEqual(actions[1].cell, "A1")

    def test_human_hand_obstacle_can_pause_when_target_blocked(self):
        from src.common.config import DEFAULT_CONFIG
        from src.common.types import Obstacle
        from src.planning.obstacle_map import assess_obstacle_intervention

        target_xyz = DEFAULT_CONFIG.human_hand_zone_center
        blocking_hand = Obstacle(
            obstacle_id="human_hand_zone",
            center_xyz=target_xyz,
            radius=DEFAULT_CONFIG.human_hand_planning_radius,
            height=DEFAULT_CONFIG.human_hand_zone_radius * 2.0,
            dynamic=True,
        )

        decision = assess_obstacle_intervention(target_xyz, [blocking_hand], DEFAULT_CONFIG)

        self.assertEqual(decision.status, "pause")
        self.assertIn("blocked", decision.reason)

    def test_human_hand_zone_geometry_is_shared_by_planning_and_simulation(self):
        import math

        from src.common.config import DEFAULT_CONFIG
        from src.planning.obstacle_map import build_obstacle_map
        from src.simulation.scene_builder import _human_safety_zone_geometry

        obstacle = build_obstacle_map(
            piece_cells=[],
            extra_obstacles=[],
            human_hand_present=True,
            config=DEFAULT_CONFIG,
        )[0]
        center, visual_radius, visual_length, orientation_rpy = _human_safety_zone_geometry(DEFAULT_CONFIG)

        self.assertEqual(center, DEFAULT_CONFIG.human_hand_zone_center)
        self.assertEqual(obstacle.center_xyz, center)
        self.assertEqual(visual_radius, DEFAULT_CONFIG.human_hand_zone_radius)
        self.assertEqual(visual_length, DEFAULT_CONFIG.human_hand_zone_length)
        # 人手区已改为 HORIZONTAL_CYLINDER 精确建模：
        # radius = 横截面半径 (human_hand_zone_radius), height = 长度 (human_hand_zone_length)
        self.assertEqual(obstacle.radius, DEFAULT_CONFIG.human_hand_zone_radius)
        self.assertEqual(obstacle.height, DEFAULT_CONFIG.human_hand_zone_length)
        self.assertEqual(orientation_rpy, (0.0, math.pi / 2.0, 0.0))

    def test_main_demo_accepts_obstacle_mode_without_piece_attachment(self):
        from main import run_demo

        result = run_demo("obstacle_mode 2")

        self.assertEqual(result["command"].command_type, "obstacle_mode")
        self.assertGreaterEqual(result["trajectory_points"], 1)
        self.assertTrue(result["execution"].success)

    def test_gui_poll_command_returns_none_for_empty_input_and_quit_command(self):
        from src.interaction.board_state import create_initial_board
        from src.interaction.gui import poll_gui_command

        board = create_initial_board()

        self.assertIsNone(poll_gui_command(board, input_func=lambda prompt: ""))
        self.assertEqual(poll_gui_command(board, input_func=lambda prompt: "quit").command_type, "quit")

    def test_gui_poll_command_parses_coordinate_and_chinese_input(self):
        from src.interaction.board_state import create_initial_board
        from src.interaction.gui import poll_gui_command

        board = create_initial_board()
        board.pieces["B1"] = board.pieces.pop("A1")
        board.pieces["B1"] = type(board.pieces["B1"])(
            piece_id="red_rook_2",
            kind=board.pieces["B1"].kind,
            color=board.pieces["B1"].color,
            cell="B1",
        )

        coordinate = poll_gui_command(board, input_func=lambda prompt: "A1 B1")
        chinese = poll_gui_command(board, input_func=lambda prompt: chinese_rook_move())

        self.assertEqual(coordinate.from_cell, "A1")
        self.assertEqual(coordinate.to_cell, "B1")
        self.assertEqual(chinese.from_cell, "B1")
        self.assertEqual(chinese.to_cell, "G1")

    def test_interactive_mode_processes_multiple_commands_in_one_session(self):
        from main import run_interactive

        commands = iter(["A1 A2", "obstacle_mode 2", "quit"])
        messages: list[str] = []

        session = run_interactive(
            input_func=lambda prompt: next(commands),
            output_func=messages.append,
            enable_board_gui=False,
        )

        self.assertEqual([result["command"].command_type for result in session["results"]], ["move", "obstacle_mode"])
        self.assertEqual(session["board"].pieces["A2"].piece_id, "red_rook_1")
        self.assertTrue(any("interactive session ended" in message for message in messages))

    def test_interactive_mode_threads_hand_state_into_obstacle_map(self):
        from main import run_interactive

        commands = iter(["hand_on", "A1 A2", "hand_off", "reset", "quit"])

        session = run_interactive(
            input_func=lambda prompt: next(commands),
            output_func=lambda message: None,
            enable_board_gui=False,
        )
        results = session["results"]

        self.assertEqual(
            [result["command"].command_type for result in results],
            ["hand_on", "move", "hand_off", "reset"],
        )
        self.assertEqual(
            [result.get("human_hand_present") for result in results],
            [True, True, False, False],
        )
        self.assertIn("human_hand_zone", results[0].get("obstacle_ids", []))
        self.assertIn("human_hand_zone", results[1].get("obstacle_ids", []))
        self.assertNotIn("human_hand_zone", results[2].get("obstacle_ids", []))
        self.assertFalse(session["human_hand_present"])

    def test_interactive_mode_toggles_human_safety_zone_visual(self):
        import main

        commands = iter(["hand_on", "hand_off", "quit"])
        visual_calls: list[bool] = []
        original_toggle = main.set_human_safety_zone
        try:
            main.set_human_safety_zone = lambda hand_present, config=main.DEFAULT_CONFIG: visual_calls.append(hand_present)

            main.run_interactive(
                input_func=lambda prompt: next(commands),
                output_func=lambda message: None,
                enable_board_gui=False,
            )
        finally:
            main.set_human_safety_zone = original_toggle

        self.assertEqual(visual_calls, [True, False])

    def test_interactive2_mode_allows_rook_jump_validation(self):
        import main
        from src.common.types import ExecutionResult, MoveCommand, RobotHandle, SceneHandle

        commands = iter([
            MoveCommand(command_type="move", from_cell="A1", to_cell="A4"),
            MoveCommand(command_type="quit"),
        ])
        allow_flags: list[object] = []
        original_poll = main.poll_gui_command
        original_run_command = main.run_command
        original_load_robot = main.load_robot
        original_build_scene = main.build_scene
        original_pump = main.set_simulation_pump_callback

        def fake_run_command(command, board, scene, robot, config=main.DEFAULT_CONFIG, **kwargs):
            allow_flags.append(kwargs.get("allow_rook_jumps"))
            return {
                "command": command,
                "actions": [],
                "execution": ExecutionResult(
                    success=True,
                    desired_joint_angles=[],
                    actual_joint_angles=[],
                    joint_errors=[],
                    end_effector_errors=[],
                    obstacle_clearances=[],
                    execution_time=0.0,
                ),
                "obstacle_ids": [],
            }

        try:
            main.poll_gui_command = lambda board, input_func=input: next(commands)
            main.run_command = fake_run_command
            main.load_robot = lambda: RobotHandle(robot_id=1, end_effector_id=2, joint_indices=())
            main.build_scene = lambda config=main.DEFAULT_CONFIG, obstacle_mode="mode_1": SceneHandle(
                board_id=1, piece_ids={}, obstacles=[]
            )
            main.set_simulation_pump_callback = lambda callback: None

            main.run_interactive(enable_board_gui=False, interactive2_mode=True)
        finally:
            main.poll_gui_command = original_poll
            main.run_command = original_run_command
            main.load_robot = original_load_robot
            main.build_scene = original_build_scene
            main.set_simulation_pump_callback = original_pump

        self.assertEqual(allow_flags, [True])

    def test_interactive_reset_replays_history_in_reverse_and_updates_gui(self):
        import main
        from src.common.types import ExecutionResult, LogicalAction, MoveCommand, RobotHandle, SceneHandle

        commands = iter([
            MoveCommand(command_type="move", from_cell="A1", to_cell="B1"),
            MoveCommand(command_type="reset"),
            MoveCommand(command_type="quit"),
        ])
        move_actions = [
            LogicalAction(action_type="pick", cell="B1", piece_id="red_horse_1"),
            LogicalAction(action_type="place", cell="CAPTURED_RED_1", piece_id="red_horse_1"),
            LogicalAction(action_type="pick", cell="A1", piece_id="red_rook_1"),
            LogicalAction(action_type="place", cell="B1", piece_id="red_rook_1"),
        ]
        expected_reset_actions = [
            LogicalAction(action_type="pick", cell="B1", piece_id="red_rook_1"),
            LogicalAction(action_type="place", cell="A1", piece_id="red_rook_1"),
            LogicalAction(action_type="pick", cell="CAPTURED_RED_1", piece_id="red_horse_1"),
            LogicalAction(action_type="place", cell="B1", piece_id="red_horse_1"),
        ]
        reset_overrides: list[list[LogicalAction] | None] = []

        class FakeBoardGUI:
            is_open = True

            def __init__(self):
                self.snapshots: list[dict[str, str]] = []

            def update_board(self, board):
                self.snapshots.append({
                    cell: piece.piece_id for cell, piece in board.pieces.items()
                })

            def set_status(self, text):
                pass

            def log(self, line):
                pass

            def set_obstacle_mode(self, mode):
                pass

            def close(self):
                self.is_open = False

        fake_gui = FakeBoardGUI()
        original_poll = main.poll_gui_command
        original_create_gui = main.create_board_gui
        original_run_command = main.run_command
        original_load_robot = main.load_robot
        original_build_scene = main.build_scene
        original_pump = main.set_simulation_pump_callback

        def fake_run_command(command, board, scene, robot, config=main.DEFAULT_CONFIG, **kwargs):
            actions = move_actions
            if command.command_type == "reset":
                actions = kwargs.get("logical_actions_override")
                reset_overrides.append(actions)
                if actions is None:
                    actions = []
            main.apply_logical_actions(board, actions)
            return {
                "command": command,
                "actions": actions,
                "execution": ExecutionResult(
                    success=True,
                    desired_joint_angles=[],
                    actual_joint_angles=[],
                    joint_errors=[],
                    end_effector_errors=[],
                    obstacle_clearances=[],
                    execution_time=0.0,
                ),
                "obstacle_ids": [],
            }

        try:
            main.poll_gui_command = lambda board, input_func=input: next(commands)
            main.create_board_gui = lambda board: fake_gui
            main.run_command = fake_run_command
            main.load_robot = lambda: RobotHandle(robot_id=1, end_effector_id=2, joint_indices=())
            main.build_scene = lambda config=main.DEFAULT_CONFIG, obstacle_mode="mode_1": SceneHandle(
                board_id=1, piece_ids={}, obstacles=[]
            )
            main.set_simulation_pump_callback = lambda callback: None

            main.run_interactive(enable_board_gui=True)
        finally:
            main.poll_gui_command = original_poll
            main.create_board_gui = original_create_gui
            main.run_command = original_run_command
            main.load_robot = original_load_robot
            main.build_scene = original_build_scene
            main.set_simulation_pump_callback = original_pump

        self.assertEqual(reset_overrides, [expected_reset_actions])
        self.assertEqual(fake_gui.snapshots[-1]["A1"], "red_rook_1")
        self.assertEqual(fake_gui.snapshots[-1]["B1"], "red_horse_1")

    def test_interactive_simulation_pump_uses_board_gui_pump_events(self):
        import main
        from src.common.types import ExecutionResult, MoveCommand, RobotHandle, SceneHandle

        commands = iter([
            MoveCommand(command_type="move", from_cell="A1", to_cell="A2"),
            MoveCommand(command_type="quit"),
        ])
        captured_callback = {"value": None}

        class FakePlt:
            def pause(self, seconds):
                pass

        class FakeBoardGUI:
            is_open = True

            def __init__(self):
                self._plt = FakePlt()
                self.pump_calls: list[float] = []

            def pump_events(self, min_interval=0.0):
                self.pump_calls.append(min_interval)

            def update_board(self, board):
                pass

            def set_status(self, text):
                pass

            def log(self, line):
                pass

            def set_obstacle_mode(self, mode):
                pass

            def close(self):
                self.is_open = False

        fake_gui = FakeBoardGUI()
        original_poll = main.poll_gui_command
        original_create_gui = main.create_board_gui
        original_run_command = main.run_command
        original_load_robot = main.load_robot
        original_build_scene = main.build_scene
        original_pump = main.set_simulation_pump_callback

        def fake_set_simulation_pump_callback(callback):
            captured_callback["value"] = callback

        def fake_run_command(command, board, scene, robot, config=main.DEFAULT_CONFIG, **kwargs):
            callback = captured_callback["value"]
            if callback is not None:
                callback()
                callback()
            return {
                "command": command,
                "actions": [],
                "execution": ExecutionResult(
                    success=True,
                    desired_joint_angles=[],
                    actual_joint_angles=[],
                    joint_errors=[],
                    end_effector_errors=[],
                    obstacle_clearances=[],
                    execution_time=0.0,
                ),
                "obstacle_ids": [],
            }

        try:
            main.poll_gui_command = lambda board, input_func=input: next(commands)
            main.create_board_gui = lambda board: fake_gui
            main.run_command = fake_run_command
            main.load_robot = lambda: RobotHandle(robot_id=1, end_effector_id=2, joint_indices=())
            main.build_scene = lambda config=main.DEFAULT_CONFIG, obstacle_mode="mode_1": SceneHandle(
                board_id=1, piece_ids={}, obstacles=[]
            )
            main.set_simulation_pump_callback = fake_set_simulation_pump_callback

            main.run_interactive(enable_board_gui=True)
        finally:
            main.poll_gui_command = original_poll
            main.create_board_gui = original_create_gui
            main.run_command = original_run_command
            main.load_robot = original_load_robot
            main.build_scene = original_build_scene
            main.set_simulation_pump_callback = original_pump

        self.assertEqual(fake_gui.pump_calls, [0.2, 0.2])

    def test_red_chinese_notation_rook_horizontal_move(self):
        from src.common.types import BoardState, Piece, PieceColor, PieceType
        from src.interaction.chinese_notation import parse_chinese_move

        board = BoardState(
            pieces={
                "B1": Piece(piece_id="red_rook_2", kind=PieceType.ROOK, color=PieceColor.RED, cell="B1"),
            }
        )

        command = parse_chinese_move(chinese_rook_move(), board)

        self.assertEqual(command.command_type, "move")
        self.assertEqual(command.from_cell, "B1")
        self.assertEqual(command.to_cell, "G1")

    def test_red_chinese_notation_cannon_advance(self):
        from src.common.types import BoardState, Piece, PieceColor, PieceType
        from src.interaction.chinese_notation import parse_chinese_move

        board = BoardState(
            pieces={
                "E3": Piece(piece_id="red_cannon_5", kind=PieceType.CANNON, color=PieceColor.RED, cell="E3"),
            }
        )

        command = parse_chinese_move(chinese_cannon_move(), board)

        self.assertEqual(command.from_cell, "E3")
        self.assertEqual(command.to_cell, "E7")

    def test_red_chinese_notation_requires_front_or_back_when_ambiguous(self):
        from src.common.types import BoardState, Piece, PieceColor, PieceType
        from src.interaction.chinese_notation import parse_chinese_move

        board = BoardState(
            pieces={
                "B1": Piece(piece_id="red_rook_back", kind=PieceType.ROOK, color=PieceColor.RED, cell="B1"),
                "B4": Piece(piece_id="red_rook_front", kind=PieceType.ROOK, color=PieceColor.RED, cell="B4"),
            }
        )

        with self.assertRaisesRegex(ValueError, "ambiguous"):
            parse_chinese_move(chinese_rook_move(), board)

        command = parse_chinese_move(chinese_front_rook_move(), board)
        self.assertEqual(command.from_cell, "B4")
        self.assertEqual(command.to_cell, "G4")

    def test_red_chinese_notation_rook_retreat(self):
        from src.common.types import BoardState, Piece, PieceColor, PieceType
        from src.interaction.chinese_notation import parse_chinese_move

        board = BoardState(
            pieces={
                "B3": Piece(piece_id="red_rook_2", kind=PieceType.ROOK, color=PieceColor.RED, cell="B3"),
            }
        )

        command = parse_chinese_move(chinese_rook_retreat(), board)

        self.assertEqual(command.command_type, "move")
        self.assertEqual(command.from_cell, "B3")
        self.assertEqual(command.to_cell, "B2")

    def test_red_chinese_notation_cannon_retreat(self):
        from src.common.types import BoardState, Piece, PieceColor, PieceType
        from src.interaction.chinese_notation import parse_chinese_move

        board = BoardState(
            pieces={
                "E4": Piece(piece_id="red_cannon_5", kind=PieceType.CANNON, color=PieceColor.RED, cell="E4"),
            }
        )

        command = parse_chinese_move(chinese_cannon_retreat(), board)

        self.assertEqual(command.from_cell, "E4")
        self.assertEqual(command.to_cell, "E2")

    def test_black_chinese_notation_rook_horizontal(self):
        from src.common.types import BoardState, Piece, PieceColor, PieceType
        from src.interaction.chinese_notation import parse_chinese_move

        board = BoardState(
            pieces={
                "I5": Piece(piece_id="black_rook_1", kind=PieceType.ROOK, color=PieceColor.BLACK, cell="I5"),
            }
        )

        command = parse_chinese_move(chinese_black_rook_horizontal(), board, side="black")

        self.assertEqual(command.command_type, "move")
        self.assertEqual(command.from_cell, "I5")
        self.assertEqual(command.to_cell, "A5")

    def test_black_chinese_notation_cannon_advance(self):
        from src.common.types import BoardState, Piece, PieceColor, PieceType
        from src.interaction.chinese_notation import parse_chinese_move

        board = BoardState(
            pieces={
                "E5": Piece(piece_id="black_cannon_5", kind=PieceType.CANNON, color=PieceColor.BLACK, cell="E5"),
            }
        )

        command = parse_chinese_move(chinese_black_cannon_advance(), board, side="black")

        self.assertEqual(command.from_cell, "E5")
        self.assertEqual(command.to_cell, "E1")

    def test_black_chinese_notation_front_rook_disambiguation(self):
        from src.common.types import BoardState, Piece, PieceColor, PieceType
        from src.interaction.chinese_notation import parse_chinese_move

        board = BoardState(
            pieces={
                "H3": Piece(piece_id="black_rook_front", kind=PieceType.ROOK, color=PieceColor.BLACK, cell="H3"),
                "H6": Piece(piece_id="black_rook_back", kind=PieceType.ROOK, color=PieceColor.BLACK, cell="H6"),
            }
        )

        command = parse_chinese_move(chinese_black_front_rook(), board, side="black")

        # Black: 前 = lower row (closer to red side) = H3; 七 = col 2 (C)
        self.assertEqual(command.from_cell, "H3")
        self.assertEqual(command.to_cell, "C3")

    def test_red_chinese_notation_horse_advance(self):
        from src.common.types import BoardState, Piece, PieceColor, PieceType
        from src.interaction.chinese_notation import parse_chinese_move

        board = BoardState(
            pieces={
                "B1": Piece(piece_id="red_horse_2", kind=PieceType.HORSE, color=PieceColor.RED, cell="B1"),
            }
        )

        command = parse_chinese_move(chinese_horse_advance(), board)

        self.assertEqual(command.from_cell, "B1")
        self.assertEqual(command.to_cell, "B4")

    # ── Bug 修复验证测试 ──

    def test_numerical_ik_is_deterministic(self):
        """根因修复：IK 必须是 (target, seed) 的纯函数（确定）。

        旧 PyBullet IK 以机器人实时姿态作迭代种子，同一目标在不同物理姿态下
        给出不同/吸盘倾斜的解，是「第二步走棋位姿失控」的根因。统一为数值 IK 后，
        同 target+seed 多次调用必须返回完全一致的结果。
        """
        from src.common.config import DEFAULT_CONFIG
        from src.planning.ik_solver import solve_ik

        seed = DEFAULT_CONFIG.home_pose[:6]
        for target in [(0.3, -0.1, 0.1), (0.4, 0.0, 0.06), (0.2, -0.3, 0.18)]:
            r1 = solve_ik(target, DEFAULT_CONFIG, seed=seed)
            r2 = solve_ik(target, DEFAULT_CONFIG, seed=seed)
            self.assertEqual(r1, r2, f"IK 对 {target} 应确定（同 seed 结果一致）")
            self.assertEqual(len(r1), 6)

    def test_current_joint_seed_returns_six_floats(self):
        """current_joint_seed 始终返回 6 元组（PyBullet 连接时读实时关节，否则回退 home）。"""
        from src.common.config import DEFAULT_CONFIG
        from src.planning.ik_solver import current_joint_seed

        seed = current_joint_seed(DEFAULT_CONFIG)
        self.assertEqual(len(seed), 6)
        self.assertTrue(all(isinstance(v, float) for v in seed))

    def test_execute_trajectory_returns_aligned_result(self):
        """流式执行：execute_trajectory 返回的各列表长度与 waypoint 数对齐。

        替代已移除的 teleport-once 测试（_init_joint_state/reset_initialization 已删）。
        """
        from src.common.types import JointTrajectory
        from src.control.controller import execute_trajectory

        trajectory = JointTrajectory(
            joint_waypoints=[
                (0.0, 0.1, 0.2, -0.3, 0.4, 0.5),
                (0.05, 0.15, 0.25, -0.35, 0.35, 0.55),
                (0.1, 0.2, 0.3, -0.4, 0.3, 0.6),
            ],
            speed_profile=["fast", "fast", "safe"],
        )
        result = execute_trajectory(trajectory)
        n = len(trajectory.joint_waypoints)
        self.assertTrue(result.success)
        self.assertEqual(len(result.actual_joint_angles), n)
        self.assertEqual(len(result.joint_errors), n)
        self.assertEqual(len(result.end_effector_errors), n)
        self.assertEqual(len(result.obstacle_clearances), n)

    def test_pybullet_execution_paces_every_simulation_step(self):
        """Real PyBullet execution should not run all sim steps in a tight CPU burst."""
        from src.control import controller

        class FakePyBullet:
            POSITION_CONTROL = 42

            def __init__(self):
                self.control_calls = []
                self.step_count = 0

            def getJointState(self, bodyUniqueId, jointIndex, physicsClientId=None):
                return (0.0, 0.0, 0.0, 0.0)

            def setJointMotorControl2(self, **kwargs):
                self.control_calls.append(kwargs)

            def stepSimulation(self, client_id):
                self.step_count += 1

        fake_p = FakePyBullet()
        ctx = controller._PyBulletContext()
        ctx.robot_id = 1
        ctx.client_id = 2
        ctx.joint_indices = (0, 1, 2, 3, 4, 5)

        pace_calls = []
        original_p = controller.p
        original_sync = controller.sync_manual_attachments
        original_pump = controller._pump_callback
        had_pace = hasattr(controller, "_pace_pybullet_realtime_step")
        original_pace = getattr(controller, "_pace_pybullet_realtime_step", None)

        try:
            controller.p = fake_p
            controller.sync_manual_attachments = lambda client_id: None
            controller._pump_callback = None
            controller._pace_pybullet_realtime_step = lambda step_started_at: pace_calls.append(step_started_at)

            controller._execute_pybullet_step(
                ctx,
                waypoint=(0.02, 0.0, 0.0, 0.0, 0.0, 0.0),
                mode="fast",
                is_last=False,
            )
        finally:
            controller.p = original_p
            controller.sync_manual_attachments = original_sync
            controller._pump_callback = original_pump
            if had_pace:
                controller._pace_pybullet_realtime_step = original_pace
            else:
                delattr(controller, "_pace_pybullet_realtime_step")

        self.assertGreater(fake_p.step_count, 0)
        self.assertEqual(len(pace_calls), fake_p.step_count)
        self.assertTrue(fake_p.control_calls)
        self.assertTrue(all(
            call["maxVelocity"] == controller._MAX_VELOCITY_FAST
            for call in fake_p.control_calls
        ))

    def test_ik_solver_yields_downward_orientation_at_e1(self):
        """Bug 2: E1 位置的 IK 解应使 tool0 z 轴接近竖直向下。

        E1 (col=4, row=0) 位于机械臂正前方较近处，之前仅位置 IK
        回退导致 EE 未保持竖直姿态。修复后应通过零空间优化
        使 -zz > 0.9。
        """
        import math

        from src.common.config import DEFAULT_CONFIG
        from src.planning.chessboard_mapping import cell_to_world
        from src.planning.ik_solver import solve_ik
        from src.control.fk_solver import _get_tool0_z_axis

        e1_xyz = cell_to_world("E1", DEFAULT_CONFIG)
        solution = solve_ik(e1_xyz, DEFAULT_CONFIG)

        self.assertEqual(len(solution), 6,
                         "IK 解应包含 6 个关节角")
        self.assertTrue(all(math.isfinite(t) for t in solution),
                        "所有关节角应为有限值")

        _, _, zz = _get_tool0_z_axis(solution, DEFAULT_CONFIG)
        self.assertGreater(-zz, 0.9,
                           f"tool0 z 轴应接近竖直向下，当前 -zz={-zz:.4f}（需 > 0.9）")

    # ── PyBullet 实物仿真测试 ──

    def test_ik_solver_prefers_downward_orientation_on_back_rank(self):
        """Row-10 grasp targets should select the vertical-down IK branch."""
        import math

        from src.common.config import DEFAULT_CONFIG
        from src.planning.chessboard_mapping import cell_to_world
        from src.planning.ik_solver import solve_ik
        from src.control.fk_solver import _get_tool0_z_axis, solve_fk

        seed = DEFAULT_CONFIG.home_pose[:6]
        for cell in ("A10", "C10", "I10"):
            world = cell_to_world(cell, DEFAULT_CONFIG)
            target = (world[0], world[1], world[2] + DEFAULT_CONFIG.z_grasp)
            solution = solve_ik(target, DEFAULT_CONFIG, seed=seed)
            fk_xyz = solve_fk(solution, DEFAULT_CONFIG)
            _, _, zz = _get_tool0_z_axis(solution, DEFAULT_CONFIG)

            self.assertLess(
                math.dist(fk_xyz, target), 0.006,
                f"{cell} IK should still reach the grasp point",
            )
            self.assertGreater(
                -zz, 0.92,
                f"{cell} tool0 z axis should point nearly straight down; got {-zz:.4f}",
            )
            seed = solution

    def test_piece_attaches_and_follows_end_effector_in_pybullet(self):
        """验证棋子（含子部件三层）吸附后跟随末端执行器运动。

        测试流程：
        1. 在 DIRECT 模式创建 PyBullet 场景，加载 UR5 + 创建一个棋子
        2. 将机械臂关节置为指向棋子上方的 pose，步进使 EE 到达
        3. 调用 attach_piece 吸附棋子
        4. 将关节移到新 target（模拟 lift），步进仿真
        5. 断言棋子位置跟随 EE（位置误差 < 2cm）
        6. 调用 detach_piece 释放，恢复质量/碰撞
        """
        import math
        import os

        from src.simulation._runtime import pybullet_available

        if not pybullet_available():
            self.skipTest("PyBullet 不可用")

        # 强制 DIRECT 模式（无 GUI）
        os.environ["CHESS_ROBOT_PYBULLET_GUI"] = "0"

        from src.common.config import DEFAULT_CONFIG
        from src.planning.chessboard_mapping import cell_to_world
        from src.simulation._runtime import RUNTIME, clear_scene_bodies, ensure_client, p
        from src.simulation.attachment import sync_manual_attachments

        # ── 隔离：断开可能由其他测试残留的 PyBullet 连接 ──
        if RUNTIME.client_id is not None:
            try:
                p.disconnect(RUNTIME.client_id)
            except Exception:
                pass
        RUNTIME.client_id = None
        RUNTIME.robot_id = None
        RUNTIME.end_effector_id = None
        RUNTIME.joint_indices = ()
        RUNTIME.scene_body_ids.clear()
        RUNTIME.piece_body_ids.clear()
        RUNTIME.piece_cells.clear()
        RUNTIME.piece_ids_by_cell.clear()
        RUNTIME.attachment_constraints.clear()
        RUNTIME.manually_attached_pieces.clear()

        client_id = ensure_client()
        self.assertIsNotNone(client_id, "无法创建 PyBullet 客户端")

        try:
            # ── 加载机器人 ──
            from src.simulation.load_robot import load_robot

            robot = load_robot()
            self.assertIsNotNone(RUNTIME.robot_id)

            # ── 创建一个测试棋子（单层主 body，无棋盘） ──
            config = DEFAULT_CONFIG
            cell = "A4"
            x, y, z = cell_to_world(cell, config)
            piece_radius = config.piece_radius
            piece_height = config.piece_height

            visual = p.createVisualShape(
                p.GEOM_CYLINDER,
                radius=piece_radius,
                length=piece_height,
                rgbaColor=(0.8, 0.6, 0.4, 1.0),
                physicsClientId=client_id,
            )
            collision = p.createCollisionShape(
                p.GEOM_CYLINDER,
                radius=piece_radius,
                height=piece_height,
                physicsClientId=client_id,
            )
            piece_body = p.createMultiBody(
                baseMass=0.02,
                baseCollisionShapeIndex=collision,
                baseVisualShapeIndex=visual,
                basePosition=(x, y, z + piece_height / 2.0),
                physicsClientId=client_id,
            )
            piece_id = "test_piece"
            RUNTIME.piece_body_ids[piece_id] = piece_body
            RUNTIME.scene_body_ids.append(piece_body)

            # ── 创建文字标签盘（简化为圆柱 + 标签两层结构） ──
            label_vis = p.createVisualShape(
                p.GEOM_CYLINDER,
                radius=piece_radius * 0.6,
                length=0.002,
                rgbaColor=(0.1, 0.1, 0.1, 1.0),
                physicsClientId=client_id,
            )
            label_id = p.createMultiBody(
                baseMass=0.001,
                baseCollisionShapeIndex=-1,
                baseVisualShapeIndex=label_vis,
                basePosition=(x, y, z + piece_height + 0.004),
                physicsClientId=client_id,
            )
            label_cid = p.createConstraint(
                parentBodyUniqueId=piece_body,
                parentLinkIndex=-1,
                childBodyUniqueId=label_id,
                childLinkIndex=-1,
                jointType=p.JOINT_FIXED,
                jointAxis=(0.0, 0.0, 0.0),
                parentFramePosition=(0.0, 0.0, piece_height / 2.0 + 0.004),
                childFramePosition=(0.0, 0.0, 0.0),
                physicsClientId=client_id,
            )
            RUNTIME.scene_body_ids.append(label_id)
            RUNTIME.attachment_constraints[f"{piece_id}_label"] = label_cid

            # ── 确认棋子初始在棋盘高度 ──
            initial_pos = p.getBasePositionAndOrientation(piece_body, physicsClientId=client_id)[0]
            self.assertAlmostEqual(initial_pos[2], z + piece_height / 2.0, delta=0.005,
                                   msg="棋子初始应在棋盘高度")

            # ── 移动机械臂到棋子正上方（模拟 approach + descend 完成后的 pose） ──
            from src.planning.ik_solver import solve_ik

            grasp_target = (x, y, config.z_grasp)  # z_grasp ≈ 0.055m above board
            grasp_joints = solve_ik(grasp_target, config)
            # 用 IK 结果的关节角初始化电机 + 仿真步进使 EE 到达 grasp pose
            for idx, joint_idx in enumerate(robot.joint_indices):
                p.setJointMotorControl2(
                    bodyUniqueId=RUNTIME.robot_id,
                    jointIndex=joint_idx,
                    controlMode=p.POSITION_CONTROL,
                    targetPosition=grasp_joints[idx],
                    force=1000,
                    maxVelocity=10.0,
                    positionGain=1.2,
                    velocityGain=0.8,
                    physicsClientId=client_id,
                )
                p.resetJointState(RUNTIME.robot_id, joint_idx, targetValue=grasp_joints[idx],
                                  physicsClientId=client_id)
            for _ in range(480):
                p.stepSimulation(client_id)
                sync_manual_attachments(client_id)

            ee_state = p.getLinkState(RUNTIME.robot_id, robot.end_effector_id, physicsClientId=client_id)
            ee_z_before = ee_state[0][2]
            self.assertLess(abs(ee_z_before - config.z_grasp), 0.03,
                            f"EE 应接近 grasp 高度，实际 z={ee_z_before:.4f}")

            # ── 吸附棋子 ──
            from src.simulation.attachment import attach_piece, detach_piece, _get_all_piece_body_ids, sync_manual_attachments

            result = attach_piece(piece_id=piece_id, end_effector_id=robot.end_effector_id)
            self.assertTrue(result.success, f"attach 失败: {result.message}")

            # 验证所有层都被找到（主 body + 标签盘 = 2 层）
            all_ids = _get_all_piece_body_ids(piece_id, client_id)
            self.assertIn(piece_body, all_ids, "主 body 应在列表中")
            self.assertIn(label_id, all_ids, "标签盘应在列表中")
            self.assertEqual(len(all_ids), 2, "应有 2 个 body（主 + label）")

            # ── 验证子部件质量已改为正数（动态） ──
            for bid in all_ids:
                dyn = p.getDynamicsInfo(bid, -1, physicsClientId=client_id)
                self.assertGreater(dyn[0], 0.0,
                                   f"body {bid} 质量应为正（动态），实际 mass={dyn[0]}")

            # ── 移动机械臂到 lift 高度（模拟吸起棋子） ──
            # 不使用 resetJointState，让关节平滑移动到目标，
            # 避免 EE 瞬跳导致约束 solver 难以收敛
            lift_target = (x, y, config.z_safe)  # z_safe ≈ 0.18m
            lift_joints = solve_ik(lift_target, config)
            for idx, joint_idx in enumerate(robot.joint_indices):
                p.setJointMotorControl2(
                    bodyUniqueId=RUNTIME.robot_id,
                    jointIndex=joint_idx,
                    controlMode=p.POSITION_CONTROL,
                    targetPosition=lift_joints[idx],
                    force=1000,
                    maxVelocity=10.0,
                    positionGain=1.2,
                    velocityGain=0.8,
                    physicsClientId=client_id,
                )
            # 步进仿真让约束拉动棋子（更多步数确保标签盘约束充分沉降）
            for _ in range(480):
                p.stepSimulation(client_id)
                sync_manual_attachments(client_id)

            # ── 验证棋子跟随 EE 到达 lift 高度 ──
            # 棋子顶部通过约束锚定在吸盘尖端（pad tip），
            # 吸盘尖端 = EE 原点 + R_ee * (0,0,suction_cup_length)
            # 棋子中心 = 吸盘尖端 - (0,0,piece_height/2)
            from src.simulation.attachment import _transform_point

            ee_lift = p.getLinkState(RUNTIME.robot_id, robot.end_effector_id, physicsClientId=client_id)
            piece_lift = p.getBasePositionAndOrientation(piece_body, physicsClientId=client_id)[0]

            pad_tip_lift = _transform_point(
                (0.0, 0.0, config.suction_cup_length), ee_lift[0], ee_lift[1]
            )
            expected_piece_lift = (
                pad_tip_lift[0],
                pad_tip_lift[1],
                pad_tip_lift[2] - config.piece_height / 2.0,
            )
            piece_error = math.hypot(
                expected_piece_lift[0] - piece_lift[0],
                expected_piece_lift[1] - piece_lift[1],
                expected_piece_lift[2] - piece_lift[2],
            )

            self.assertLess(piece_error, 0.005,
                            f"棋子中心应在吸盘尖端下方: pad_tip=({pad_tip_lift[0]:.4f},{pad_tip_lift[1]:.4f},{pad_tip_lift[2]:.4f}), "
                            f"expected piece=({expected_piece_lift[0]:.4f},{expected_piece_lift[1]:.4f},{expected_piece_lift[2]:.4f}), "
                            f"actual piece=({piece_lift[0]:.4f},{piece_lift[1]:.4f},{piece_lift[2]:.4f}), error={piece_error:.4f}m")
            self.assertGreater(piece_lift[2], config.z_grasp + 0.03,
                               f"棋子应在 grasp 高度之上: z={piece_lift[2]:.4f}")

            # ── 验证标签盘也跟随主 body（JOINT_FIXED 约束） ──
            label_pos = p.getBasePositionAndOrientation(label_id, physicsClientId=client_id)[0]
            expected_label_z = piece_lift[2] + piece_height / 2.0 + 0.004
            label_error = math.hypot(
                piece_lift[0] - label_pos[0],
                piece_lift[1] - label_pos[1],
                expected_label_z - label_pos[2],
            )
            self.assertLess(label_error, 0.03,
                            f"标签盘应跟随主 body: error={label_error:.4f}m")

            # ── 移动机械臂到目标 cell（模拟 transfer） ──
            target_cell = "A5"
            tx, ty, tz = cell_to_world(target_cell, config)
            transfer_target = (tx, ty, config.z_safe)
            transfer_joints = solve_ik(transfer_target, config)
            for idx, joint_idx in enumerate(robot.joint_indices):
                p.setJointMotorControl2(
                    bodyUniqueId=RUNTIME.robot_id,
                    jointIndex=joint_idx,
                    controlMode=p.POSITION_CONTROL,
                    targetPosition=transfer_joints[idx],
                    force=1000,
                    maxVelocity=10.0,
                    positionGain=1.2,
                    velocityGain=0.8,
                    physicsClientId=client_id,
                )
            for _ in range(480):
                p.stepSimulation(client_id)
                sync_manual_attachments(client_id)

            # ── 验证棋子跟随 EE 到达 transfer 目标 ──
            # 棋子顶部通过约束锚定在吸盘尖端，
            # 使用 _transform_point 考虑 EE 方向
            ee_transfer = p.getLinkState(RUNTIME.robot_id, robot.end_effector_id, physicsClientId=client_id)
            piece_transfer = p.getBasePositionAndOrientation(piece_body, physicsClientId=client_id)[0]

            pad_tip_transfer = _transform_point(
                (0.0, 0.0, config.suction_cup_length), ee_transfer[0], ee_transfer[1]
            )
            expected_piece_transfer = (
                pad_tip_transfer[0],
                pad_tip_transfer[1],
                pad_tip_transfer[2] - config.piece_height / 2.0,
            )
            transfer_error = math.hypot(
                expected_piece_transfer[0] - piece_transfer[0],
                expected_piece_transfer[1] - piece_transfer[1],
                expected_piece_transfer[2] - piece_transfer[2],
            )
            self.assertLess(transfer_error, 0.005,
                            f"棋子应跟随吸盘尖端: pad_tip=({pad_tip_transfer[0]:.4f},{pad_tip_transfer[1]:.4f},{pad_tip_transfer[2]:.4f}), "
                            f"expected piece=({expected_piece_transfer[0]:.4f},{expected_piece_transfer[1]:.4f},{expected_piece_transfer[2]:.4f}), "
                            f"actual piece=({piece_transfer[0]:.4f},{piece_transfer[1]:.4f},{piece_transfer[2]:.4f}), error={transfer_error:.4f}m")

            # ── 释放棋子 ──
            detach_result = detach_piece(piece_id=piece_id)
            self.assertTrue(detach_result.success, f"detach 失败: {detach_result.message}")

            # 手动吸附映射应已清除
            self.assertNotIn(piece_id, RUNTIME.manually_attached_pieces,
                             "手动吸附映射应已清除")
            # 旧版约束也应已清除（兼容）
            self.assertNotIn(piece_id, RUNTIME.attachment_constraints,
                             "piece 吸附约束应已移除")

            # ── 验证动力学恢复 ──
            main_dyn = p.getDynamicsInfo(piece_body, -1, physicsClientId=client_id)
            self.assertAlmostEqual(main_dyn[0], 0.02, delta=0.001,
                                   msg="主 body 质量应恢复为 0.02")

            # 标签盘应保持动态（mass=0.001），确保跟随主 body 下落
            label_dyn = p.getDynamicsInfo(label_id, -1, physicsClientId=client_id)
            self.assertAlmostEqual(label_dyn[0], 0.001, delta=0.001,
                                   msg="标签盘应保持动态质量（0.001），避免静态拉扯导致浮空")

        finally:
            # 清理：断开 PyBullet 连接
            cid = RUNTIME.client_id
            if cid is not None:
                try:
                    p.disconnect(cid)
                except Exception:
                    pass
            # 重置 RUNTIME 状态
            RUNTIME.client_id = None
            RUNTIME.robot_id = None
            RUNTIME.end_effector_id = None
            RUNTIME.joint_indices = ()
            RUNTIME.scene_body_ids.clear()
            RUNTIME.piece_body_ids.clear()
            RUNTIME.piece_cells.clear()
            RUNTIME.piece_ids_by_cell.clear()
            RUNTIME.attachment_constraints.clear()
            RUNTIME.manually_attached_pieces.clear()


    # ── P1: Chess piece movement rule tests ──

    def test_general_legal_moves_within_palace(self):
        """帅/将：九宫内合法的一步直行"""
        from src.common.types import BoardState, Piece, PieceColor, PieceType
        from src.interaction.chess_rules import validate_move
        from src.interaction.cli import parse_command

        # Each call to validate_move is independent; board is not mutated.
        # Place the piece at the from_cell for each test.
        board = BoardState(pieces={
            "E1": Piece(piece_id="red_general", kind=PieceType.GENERAL, color=PieceColor.RED, cell="E1"),
            "E10": Piece(piece_id="black_general", kind=PieceType.GENERAL, color=PieceColor.BLACK, cell="E10"),
        })

        # Red general from E1 (col=4, row=0): forward, left, right
        self.assertTrue(validate_move(board, parse_command("E1 E2")).is_legal, "forward one step")
        self.assertTrue(validate_move(board, parse_command("E1 D1")).is_legal, "left one step")
        self.assertTrue(validate_move(board, parse_command("E1 F1")).is_legal, "right one step")

        # Red general from E2 (row=1): forward, left, right
        board2 = BoardState(pieces={
            "E2": Piece(piece_id="red_general", kind=PieceType.GENERAL, color=PieceColor.RED, cell="E2"),
        })
        self.assertTrue(validate_move(board2, parse_command("E2 E3")).is_legal, "forward from E2")
        self.assertTrue(validate_move(board2, parse_command("E2 D2")).is_legal, "left from E2")
        self.assertTrue(validate_move(board2, parse_command("E2 F2")).is_legal, "right from E2")

        # Black general: one step within palace
        self.assertTrue(validate_move(board, parse_command("E10 E9")).is_legal, "black forward one step")
        self.assertTrue(validate_move(board, parse_command("E10 D10")).is_legal, "black left one step")

    def test_general_illegal_move_outside_palace(self):
        """帅/将：禁止出九宫"""
        from src.common.types import BoardState, Piece, PieceColor, PieceType
        from src.interaction.chess_rules import validate_move
        from src.interaction.cli import parse_command

        board = BoardState(pieces={
            "D3": Piece(piece_id="red_general", kind=PieceType.GENERAL, color=PieceColor.RED, cell="D3"),
            "D8": Piece(piece_id="black_general", kind=PieceType.GENERAL, color=PieceColor.BLACK, cell="D8"),
        })

        # Red: D3(row=2) → D4(row=3) leaves palace (max red row is 2)
        self.assertFalse(validate_move(board, parse_command("D3 D4")).is_legal, "outside palace (row)")
        # Red: D3(col=3) → C3(col=2) leaves palace (min palace col is 3)
        self.assertFalse(validate_move(board, parse_command("D3 C3")).is_legal, "outside palace (col)")
        # Black: D8(row=7) → D7(row=6) leaves palace (min black row is 7)
        self.assertFalse(validate_move(board, parse_command("D8 D7")).is_legal, "black outside palace")

    def test_general_must_move_one_step_orthogonal(self):
        """帅/将：禁止斜行和超过一步"""
        from src.common.types import BoardState, Piece, PieceColor, PieceType
        from src.interaction.chess_rules import validate_move
        from src.interaction.cli import parse_command

        board = BoardState(pieces={
            "E1": Piece(piece_id="red_general", kind=PieceType.GENERAL, color=PieceColor.RED, cell="E1"),
        })

        # Diagonal is illegal
        self.assertFalse(validate_move(board, parse_command("E1 D2")).is_legal, "diagonal illegal")
        # Two steps is illegal
        self.assertFalse(validate_move(board, parse_command("E1 E3")).is_legal, "two steps illegal")

    def test_advisor_legal_diagonal_within_palace(self):
        """仕/士：九宫内的合法斜行"""
        from src.common.types import BoardState, Piece, PieceColor, PieceType
        from src.interaction.chess_rules import validate_move
        from src.interaction.cli import parse_command

        # validate_move does not mutate board state, so each test uses its own board.
        board1 = BoardState(pieces={
            "D1": Piece(piece_id="red_advisor_1", kind=PieceType.ADVISOR, color=PieceColor.RED, cell="D1"),
        })
        board2 = BoardState(pieces={
            "E2": Piece(piece_id="red_advisor_1", kind=PieceType.ADVISOR, color=PieceColor.RED, cell="E2"),
        })
        board3 = BoardState(pieces={
            "F10": Piece(piece_id="black_advisor_2", kind=PieceType.ADVISOR, color=PieceColor.BLACK, cell="F10"),
        })

        # Red advisor: D1 → E2 (diagonal, within palace)
        self.assertTrue(validate_move(board1, parse_command("D1 E2")).is_legal)
        # Move back: E2 → D1
        self.assertTrue(validate_move(board2, parse_command("E2 D1")).is_legal)
        # Black advisor: F10 → E9 (diagonal, within palace)
        self.assertTrue(validate_move(board3, parse_command("F10 E9")).is_legal)

    def test_advisor_illegal_straight_move(self):
        """仕/士：禁止直行"""
        from src.common.types import BoardState, Piece, PieceColor, PieceType
        from src.interaction.chess_rules import validate_move
        from src.interaction.cli import parse_command

        board = BoardState(pieces={
            "D1": Piece(piece_id="red_advisor_1", kind=PieceType.ADVISOR, color=PieceColor.RED, cell="D1"),
        })

        self.assertFalse(validate_move(board, parse_command("D1 D2")).is_legal, "straight move illegal")
        self.assertFalse(validate_move(board, parse_command("D1 E1")).is_legal, "straight move illegal")

    def test_advisor_illegal_outside_palace(self):
        """仕/士：禁止出九宫"""
        from src.common.types import BoardState, Piece, PieceColor, PieceType
        from src.interaction.chess_rules import validate_move
        from src.interaction.cli import parse_command

        board = BoardState(pieces={
            "F1": Piece(piece_id="red_advisor_2", kind=PieceType.ADVISOR, color=PieceColor.RED, cell="F1"),
            "D10": Piece(piece_id="black_advisor_1", kind=PieceType.ADVISOR, color=PieceColor.BLACK, cell="D10"),
        })

        # Red advisor at F1, diagonal to G2 is outside palace (col 6)
        self.assertFalse(validate_move(board, parse_command("F1 G2")).is_legal, "outside palace col")
        # Black advisor at D10, diagonal to C9 is outside palace (col 2)
        self.assertFalse(validate_move(board, parse_command("D10 C9")).is_legal, "outside palace col")

    def test_elephant_legal_field_move(self):
        """相/象：合法的田字对角移动"""
        from src.common.types import BoardState, Piece, PieceColor, PieceType
        from src.interaction.chess_rules import validate_move
        from src.interaction.cli import parse_command

        board = BoardState(pieces={
            "C1": Piece(piece_id="red_elephant_1", kind=PieceType.ELEPHANT, color=PieceColor.RED, cell="C1"),
            "G1": Piece(piece_id="red_elephant_2", kind=PieceType.ELEPHANT, color=PieceColor.RED, cell="G1"),
            "C10": Piece(piece_id="black_elephant_1", kind=PieceType.ELEPHANT, color=PieceColor.BLACK, cell="C10"),
        })

        # Red elephant: C1 → A3 (田字)
        self.assertTrue(validate_move(board, parse_command("C1 A3")).is_legal)
        # Red elephant: C1 → E3 (田字)
        self.assertTrue(validate_move(board, parse_command("C1 E3")).is_legal)
        # Black elephant: C10 → A8 (田字)
        self.assertTrue(validate_move(board, parse_command("C10 A8")).is_legal)
        # Black elephant: C10 → E8 (田字)
        self.assertTrue(validate_move(board, parse_command("C10 E8")).is_legal)

    def test_elephant_eye_blocked(self):
        """相/象：塞象眼时不能走"""
        from src.common.types import BoardState, Piece, PieceColor, PieceType
        from src.interaction.chess_rules import validate_move
        from src.interaction.cli import parse_command

        board = BoardState(pieces={
            "C1": Piece(piece_id="red_elephant_1", kind=PieceType.ELEPHANT, color=PieceColor.RED, cell="C1"),
            # Block the elephant eye at B2 (midpoint between C1 and A3)
            "B2": Piece(piece_id="blocker", kind=PieceType.SOLDIER, color=PieceColor.RED, cell="B2"),
        })

        self.assertFalse(validate_move(board, parse_command("C1 A3")).is_legal, "eye blocked at B2")

    def test_elephant_cannot_cross_river(self):
        """相/象：禁止过河"""
        from src.common.types import BoardState, Piece, PieceColor, PieceType
        from src.interaction.chess_rules import validate_move
        from src.interaction.cli import parse_command

        # Red elephant must stay in rows 0-4; black in rows 5-9
        board = BoardState(pieces={
            "C4": Piece(piece_id="red_elephant_1", kind=PieceType.ELEPHANT, color=PieceColor.RED, cell="C4"),
            "C5": Piece(piece_id="black_elephant_1", kind=PieceType.ELEPHANT, color=PieceColor.BLACK, cell="C5"),
        })

        # Red C4(row=3) → E6(row=5): crosses river (row 5 >= 5)
        self.assertFalse(validate_move(board, parse_command("C4 E6")).is_legal, "red crosses to row 5")
        # Black C5(row=4) → A3(row=2): crosses river (row 2 <= 4)
        self.assertFalse(validate_move(board, parse_command("C5 A3")).is_legal, "black crosses to row 2")

    def test_soldier_forward_before_crossing_river(self):
        """兵/卒：未过河只能前进"""
        from src.common.types import BoardState, Piece, PieceColor, PieceType
        from src.interaction.chess_rules import validate_move
        from src.interaction.cli import parse_command

        board = BoardState(pieces={
            "A4": Piece(piece_id="red_soldier_1", kind=PieceType.SOLDIER, color=PieceColor.RED, cell="A4"),
            "A7": Piece(piece_id="black_soldier_1", kind=PieceType.SOLDIER, color=PieceColor.BLACK, cell="A7"),
        })

        # Red soldier: forward A4 → A5 is legal
        self.assertTrue(validate_move(board, parse_command("A4 A5")).is_legal, "forward before river")
        # Black soldier: forward A7 → A6 is legal
        self.assertTrue(validate_move(board, parse_command("A7 A6")).is_legal, "black forward before river")

    def test_soldier_cannot_retreat(self):
        """兵/卒：禁止后退"""
        from src.common.types import BoardState, Piece, PieceColor, PieceType
        from src.interaction.chess_rules import validate_move
        from src.interaction.cli import parse_command

        board = BoardState(pieces={
            "A5": Piece(piece_id="red_soldier_1", kind=PieceType.SOLDIER, color=PieceColor.RED, cell="A5"),
            "A6": Piece(piece_id="black_soldier_1", kind=PieceType.SOLDIER, color=PieceColor.BLACK, cell="A6"),
        })

        # Red soldier cannot retreat (A5 → A4)
        self.assertFalse(validate_move(board, parse_command("A5 A4")).is_legal, "red cannot retreat")
        # Black soldier cannot retreat (A6 → A7)
        self.assertFalse(validate_move(board, parse_command("A6 A7")).is_legal, "black cannot retreat")

    def test_soldier_cannot_move_sideways_before_river(self):
        """兵/卒：未过河禁止左右移动"""
        from src.common.types import BoardState, Piece, PieceColor, PieceType
        from src.interaction.chess_rules import validate_move
        from src.interaction.cli import parse_command

        board = BoardState(pieces={
            "A4": Piece(piece_id="red_soldier_1", kind=PieceType.SOLDIER, color=PieceColor.RED, cell="A4"),
            "C7": Piece(piece_id="black_soldier_2", kind=PieceType.SOLDIER, color=PieceColor.BLACK, cell="C7"),
        })

        self.assertFalse(validate_move(board, parse_command("A4 B4")).is_legal, "red sideways before river")
        self.assertFalse(validate_move(board, parse_command("C7 B7")).is_legal, "black sideways before river")

    def test_soldier_sideways_and_forward_after_crossing_river(self):
        """兵/卒：过河后可左右移动和继续前进"""
        from src.common.types import BoardState, Piece, PieceColor, PieceType
        from src.interaction.chess_rules import validate_move
        from src.interaction.cli import parse_command

        # Red soldier at C6 (row=5): crossed river (row >= 5)
        # Black soldier at C3 (row=2): crossed river (row <= 4)
        board = BoardState(pieces={
            "C6": Piece(piece_id="red_soldier_2", kind=PieceType.SOLDIER, color=PieceColor.RED, cell="C6"),
            "C3": Piece(piece_id="black_soldier_2", kind=PieceType.SOLDIER, color=PieceColor.BLACK, cell="C3"),
        })

        # Red soldier (crossed): can move left, right, forward
        self.assertTrue(validate_move(board, parse_command("C6 B6")).is_legal, "red left after crossing")
        self.assertTrue(validate_move(board, parse_command("C6 D6")).is_legal, "red right after crossing")
        self.assertTrue(validate_move(board, parse_command("C6 C7")).is_legal, "red forward after crossing")
        # Red soldier still cannot retreat
        self.assertFalse(validate_move(board, parse_command("C6 C5")).is_legal, "red cannot retreat after crossing")

        # Black soldier (crossed): can move left, right, forward
        self.assertTrue(validate_move(board, parse_command("C3 B3")).is_legal, "black left after crossing")
        self.assertTrue(validate_move(board, parse_command("C3 D3")).is_legal, "black right after crossing")
        self.assertTrue(validate_move(board, parse_command("C3 C2")).is_legal, "black forward after crossing")

    # ── P5a/P5b/P5c: trajectory optimisation upgrades ──

    def test_theta_star_produces_straight_line_without_obstacles(self):
        """P5a: Theta* produces a near-straight path when no obstacles are present."""
        import math
        from src.common.config import DEFAULT_CONFIG
        from src.planning.path_search import a_star_theta_2d

        start = (0.5, 0.5)
        end = (0.5, 1.5)
        z_plane = 0.18
        result = a_star_theta_2d(
            start, end, obstacles=[], z_plane=z_plane,
            grid_resolution=0.02, timeout_ms=500.0, config=DEFAULT_CONFIG,
        )
        self.assertTrue(result.success, "Theta* should succeed on empty map")
        self.assertGreater(len(result.path_xy), 1, "should produce at least 2 waypoints")
        direct_dist = math.hypot(end[0] - start[0], end[1] - start[1])
        path_length = sum(
            math.hypot(
                result.path_xy[i + 1][0] - result.path_xy[i][0],
                result.path_xy[i + 1][1] - result.path_xy[i][1],
            )
            for i in range(len(result.path_xy) - 1)
        )
        self.assertLess(
            path_length / direct_dist, 1.10,
            f"Theta* path length ({path_length:.3f}) should be close to direct ({direct_dist:.3f})",
        )

    def test_cubic_spline_output_count(self):
        """P5b: Cubic spline output count is a requested multiple of input count."""
        from src.planning.trajectory_smoother import smooth_joint_trajectory_cubic_spline

        waypoints = [
            (0.0, 0.1, 0.2, -0.3, 0.4, 0.5),
            (0.1, 0.2, 0.3, -0.4, 0.5, 0.6),
            (0.2, 0.3, 0.4, -0.5, 0.6, 0.7),
            (0.3, 0.4, 0.5, -0.6, 0.7, 0.8),
            (0.4, 0.5, 0.6, -0.7, 0.8, 0.9),
        ]
        num_samples = len(waypoints) * 2
        smoothed = smooth_joint_trajectory_cubic_spline(waypoints, num_samples=num_samples)
        self.assertEqual(len(smoothed), num_samples)
        self.assertEqual(len(smoothed[0]), 6)
        default_smoothed = smooth_joint_trajectory_cubic_spline(waypoints)
        self.assertEqual(len(default_smoothed), len(waypoints))
        for j in range(6):
            self.assertAlmostEqual(smoothed[0][j], waypoints[0][j], delta=1e-9)
            self.assertAlmostEqual(smoothed[-1][j], waypoints[-1][j], delta=1e-9)
        short = [(1.0, 2.0, 3.0, 4.0, 5.0, 6.0)]
        self.assertEqual(smooth_joint_trajectory_cubic_spline(short, num_samples=10), short)

    def test_jerk_opt_preserves_endpoints(self):
        """P5c: Jerk optimisation keeps first and last waypoint unchanged."""
        import math
        from src.planning.trajectory_smoother import optimize_jerk_minimum

        waypoints = [
            (0.0, 0.1, 0.2, -0.3, 0.4, 0.5),
            (0.1, 0.2, 0.3, -0.4, 0.5, 0.6),
            (0.2, 0.3, 0.4, -0.5, 0.6, 0.7),
            (0.3, 0.4, 0.5, -0.6, 0.7, 0.8),
            (0.4, 0.5, 0.6, -0.7, 0.8, 0.9),
        ]
        speed_profile = ["safe"] * len(waypoints)
        optimized = optimize_jerk_minimum(waypoints, speed_profile, num_iterations=50, learning_rate=0.01)
        self.assertEqual(len(optimized), len(waypoints))
        for j in range(6):
            self.assertAlmostEqual(optimized[0][j], waypoints[0][j], delta=1e-9)
            self.assertAlmostEqual(optimized[-1][j], waypoints[-1][j], delta=1e-9)
        for wp in optimized:
            for val in wp:
                self.assertTrue(math.isfinite(val))
        short = [(1.0, 2.0, 3.0, 4.0, 5.0, 6.0)]
        self.assertEqual(optimize_jerk_minimum(short, ["safe"]), short)

    def test_plan_trajectory_accepts_new_enable_parameters(self):
        """plan_trajectory 的 enable_* 开关可独立关闭，且产出结构一致。

        注：实验性 enable_theta_star/cubic_spline/jerk_opt 已移除（已知数值缺陷，
        见控制器修复交接）。本测试覆盖现存的 path_search/smoothing/interpolation 开关。
        """
        from src.common.config import DEFAULT_CONFIG
        from src.common.types import LogicalAction
        from src.planning.motion_primitives import build_motion_primitives
        from src.planning.obstacle_map import build_obstacle_map
        from src.planning.trajectory_planner import plan_trajectory

        actions = [
            LogicalAction(action_type="pick", cell="A1"),
            LogicalAction(action_type="place", cell="B1"),
        ]
        obstacles = build_obstacle_map(piece_cells=["C1"], extra_obstacles=[])
        primitives = build_motion_primitives(actions, DEFAULT_CONFIG)
        traj_default = plan_trajectory(primitives, obstacles, DEFAULT_CONFIG)
        self.assertGreater(len(traj_default.joint_waypoints), 0)
        self.assertEqual(len(traj_default.joint_waypoints), len(traj_default.speed_profile))

        traj_off = plan_trajectory(
            primitives, obstacles, DEFAULT_CONFIG,
            enable_path_search=False, enable_smoothing=False, enable_interpolation=False,
        )
        self.assertGreater(len(traj_off.joint_waypoints), 0)
        self.assertEqual(len(traj_off.joint_waypoints), len(traj_off.speed_profile))


    # ── P2: Matplotlib Board GUI tests ──

    def test_board_gui_cell_to_grid_conversion(self):
        """BoardGUI helper: _cell_to_grid / _grid_to_cell round-trip correctly."""
        from src.interaction.board_gui import _cell_to_grid, _grid_to_cell

        # Valid cells
        self.assertEqual(_cell_to_grid("A1"), (0, 0))
        self.assertEqual(_cell_to_grid("I10"), (8, 9))
        self.assertEqual(_cell_to_grid("E5"), (4, 4))
        self.assertEqual(_cell_to_grid("a1"), (0, 0))  # lowercase
        self.assertEqual(_cell_to_grid("B3"), (1, 2))

        # Round-trip
        for cell in ("A1", "B3", "C5", "I10", "E5"):
            col, row = _cell_to_grid(cell)
            self.assertIsNotNone(col)
            self.assertIsNotNone(row)
            result = _grid_to_cell(col, row)
            self.assertEqual(result, cell)

        # Invalid / out-of-bounds
        self.assertEqual(_cell_to_grid(""), (None, None))
        self.assertEqual(_cell_to_grid("Z9"), (None, None))
        self.assertEqual(_cell_to_grid("J1"), (None, None))
        self.assertEqual(_cell_to_grid("A11"), (None, None))
        self.assertEqual(_cell_to_grid("A0"), (None, None))

    def test_board_gui_grid_to_cell_format(self):
        """BoardGUI helper: _grid_to_cell produces correct format."""
        from src.interaction.board_gui import _grid_to_cell

        self.assertEqual(_grid_to_cell(0, 0), "A1")
        self.assertEqual(_grid_to_cell(8, 9), "I10")
        self.assertEqual(_grid_to_cell(4, 4), "E5")
        self.assertEqual(_grid_to_cell(3, 7), "D8")

    def test_board_gui_piece_chars_mapping(self):
        """PIECE_CHARS covers all piece type/color combinations."""
        from src.common.types import PieceColor, PieceType
        from src.interaction.board_gui import PIECE_CHARS

        self.assertEqual(len(PIECE_CHARS), 14)  # 7 piece types x 2 colors
        self.assertEqual(PIECE_CHARS[(PieceColor.RED, PieceType.ROOK)], chr(0x8f66))
        self.assertEqual(PIECE_CHARS[(PieceColor.BLACK, PieceType.ROOK)], chr(0x8eca))
        self.assertEqual(PIECE_CHARS[(PieceColor.RED, PieceType.GENERAL)], chr(0x5e25))
        self.assertEqual(PIECE_CHARS[(PieceColor.BLACK, PieceType.GENERAL)], chr(0x5c07))
        self.assertEqual(PIECE_CHARS[(PieceColor.RED, PieceType.SOLDIER)], chr(0x5175))
        self.assertEqual(PIECE_CHARS[(PieceColor.BLACK, PieceType.SOLDIER)], chr(0x5352))

    def test_board_gui_exports_piece_chars(self):
        """PIECE_CHARS is importable from board_gui module."""
        from src.interaction.board_gui import PIECE_CHARS, BoardGUI
        self.assertIsInstance(PIECE_CHARS, dict)
        self.assertTrue(callable(BoardGUI))

    def test_board_gui_programmatic_obstacle_mode_sync_does_not_enqueue_command(self):
        """Programmatic radio sync should not echo obstacle_mode into the command queue."""
        import queue
        from src.interaction.board_gui import BoardGUI

        class FakeRadio:
            def __init__(self, callback):
                self.callback = callback
                self.active = 0

            def disconnect_events(self):
                pass

            def set_active(self, index):
                self.active = index
                labels = ("mode_1", "mode_2", "mode_3", "mode_4")
                self.callback(labels[index])

            def on_clicked(self, callback):
                self.callback = callback

        gui = BoardGUI.__new__(BoardGUI)
        gui._command_queue = queue.Queue()
        gui._current_obstacle_mode = "mode_1"
        gui._radio = FakeRadio(gui._on_obstacle_mode)

        gui.set_obstacle_mode("mode_2")

        self.assertTrue(gui._command_queue.empty())
        self.assertEqual(gui._current_obstacle_mode, "mode_2")

    def test_board_gui_graceful_import_error_without_matplotlib(self):
        """BoardGUI._ensure_matplotlib raises ImportError when matplotlib missing."""
        import sys, builtins
        from src.interaction.board_state import create_initial_board

        create_initial_board()

        saved = sys.modules.get('matplotlib')
        saved_pyplot = sys.modules.get('matplotlib.pyplot')
        try:
            sys.modules['matplotlib'] = None
            sys.modules['matplotlib.pyplot'] = None
            from src.interaction.board_gui import BoardGUI as BG
            _orig = builtins.__import__
            def _block(name, *a, **kw):
                if name == 'matplotlib' or name.startswith('matplotlib.'):
                    raise ImportError("Mocked")
                return _orig(name, *a, **kw)
            builtins.__import__ = _block
            try:
                with self.assertRaises(ImportError):
                    BG._ensure_matplotlib()
            finally:
                builtins.__import__ = _orig
        finally:
            if saved is None:
                sys.modules.pop('matplotlib', None)
            else:
                sys.modules['matplotlib'] = saved
            if saved_pyplot is None:
                sys.modules.pop('matplotlib.pyplot', None)
            else:
                sys.modules['matplotlib.pyplot'] = saved_pyplot

    def test_board_gui_creates_move_command_from_two_clicks(self):
        """Simulate two cell clicks produce a valid MoveCommand."""
        from unittest.mock import MagicMock
        from src.interaction.board_state import create_initial_board

        board = create_initial_board()
        mock_plt = MagicMock()

        mock_fig = MagicMock()
        mock_ax = MagicMock()
        mock_fig.canvas.mpl_connect = MagicMock(return_value=1)
        mock_fig.canvas.mpl_disconnect = MagicMock()
        mock_fig.canvas.manager.set_window_title = MagicMock()

        import queue
        from src.interaction.board_gui import BoardGUI
        gui = BoardGUI.__new__(BoardGUI)
        gui._plt = mock_plt
        gui._Rectangle = MagicMock()
        gui._command_queue = queue.Queue()
        gui._selected_cell = None
        gui._highlight_rect = None
        gui._hover_rect = None
        gui._board = board
        gui._window_open = True
        gui._cid_click = 1
        gui._cid_motion = 2
        gui._cid_close = 3
        gui._fig = mock_fig
        gui._ax = mock_ax
        gui._last_draw_time = 0.0
        gui._draw(board)

        # First click: select source A1
        ev1 = MagicMock()
        ev1.xdata = 0.0
        ev1.ydata = 0.0
        gui._on_click(ev1)
        self.assertEqual(gui._selected_cell, "A1")

        # Second click: select target A2
        ev2 = MagicMock()
        ev2.xdata = 0.0
        ev2.ydata = 1.0
        gui._on_click(ev2)

        cmd = gui.get_next_command()
        self.assertIsNotNone(cmd)
        self.assertEqual(cmd.command_type, "move")
        self.assertEqual(cmd.from_cell, "A1")
        self.assertEqual(cmd.to_cell, "A2")
        self.assertIsNone(gui._selected_cell)

    def test_board_gui_same_cell_click_deselects(self):
        """Clicking the same cell twice deselects without enqueuing."""
        from unittest.mock import MagicMock
        from src.interaction.board_state import create_initial_board

        board = create_initial_board()
        mock_plt = MagicMock()

        mock_fig = MagicMock()
        mock_ax = MagicMock()
        mock_fig.canvas.mpl_connect = MagicMock(return_value=1)
        mock_fig.canvas.mpl_disconnect = MagicMock()
        mock_fig.canvas.manager.set_window_title = MagicMock()

        import queue
        from src.interaction.board_gui import BoardGUI
        gui = BoardGUI.__new__(BoardGUI)
        gui._plt = mock_plt
        gui._Rectangle = MagicMock()
        gui._command_queue = queue.Queue()
        gui._selected_cell = None
        gui._highlight_rect = None
        gui._hover_rect = None
        gui._board = board
        gui._window_open = True
        gui._cid_click = 1
        gui._cid_motion = 2
        gui._cid_close = 3
        gui._fig = mock_fig
        gui._ax = mock_ax
        gui._last_draw_time = 0.0
        gui._draw(board)

        click = MagicMock()
        click.xdata = 2.0
        click.ydata = 3.0
        gui._on_click(click)
        self.assertEqual(gui._selected_cell, "C4")

        gui._on_click(click)  # same cell deselects
        self.assertIsNone(gui._selected_cell)
        self.assertIsNone(gui.get_next_command())

    def test_board_gui_click_outside_board_ignored(self):
        """Clicks outside the board grid do not produce selections or commands."""
        from unittest.mock import MagicMock
        from src.interaction.board_state import create_initial_board

        board = create_initial_board()
        mock_plt = MagicMock()

        mock_fig = MagicMock()
        mock_ax = MagicMock()
        mock_fig.canvas.mpl_connect = MagicMock(return_value=1)
        mock_fig.canvas.mpl_disconnect = MagicMock()
        mock_fig.canvas.manager.set_window_title = MagicMock()

        import queue
        from src.interaction.board_gui import BoardGUI
        gui = BoardGUI.__new__(BoardGUI)
        gui._plt = mock_plt
        gui._Rectangle = MagicMock()
        gui._command_queue = queue.Queue()
        gui._selected_cell = None
        gui._highlight_rect = None
        gui._hover_rect = None
        gui._board = board
        gui._window_open = True
        gui._cid_click = 1
        gui._cid_motion = 2
        gui._cid_close = 3
        gui._fig = mock_fig
        gui._ax = mock_ax
        gui._last_draw_time = 0.0
        gui._draw(board)

        for x, y in [(-1.0, -1.0), (9.5, 10.5)]:
            ev = MagicMock()
            ev.xdata = x
            ev.ydata = y
            gui._on_click(ev)
            self.assertIsNone(gui._selected_cell)

        self.assertIsNone(gui.get_next_command())

    def test_board_gui_update_board_redraws(self):
        """update_board() clears old state and redraws."""
        from unittest.mock import MagicMock
        from src.interaction.board_state import create_initial_board

        board = create_initial_board()
        mock_plt = MagicMock()

        mock_fig = MagicMock()
        mock_ax = MagicMock()
        mock_fig.canvas.mpl_connect = MagicMock(return_value=1)
        mock_fig.canvas.mpl_disconnect = MagicMock()
        mock_fig.canvas.manager.set_window_title = MagicMock()
        mock_fig.canvas.draw_idle = MagicMock()

        import queue
        from src.interaction.board_gui import BoardGUI
        gui = BoardGUI.__new__(BoardGUI)
        gui._plt = mock_plt
        gui._Rectangle = MagicMock()
        gui._command_queue = queue.Queue()
        gui._selected_cell = "A1"
        gui._highlight_rect = MagicMock()
        gui._hover_rect = None
        gui._board = board
        gui._window_open = True
        gui._cid_click = 1
        gui._cid_motion = 2
        gui._cid_close = 3
        gui._fig = mock_fig
        gui._ax = mock_ax
        gui._last_draw_time = 0.0
        gui._draw(board)

        gui.update_board(board)
        self.assertIsNone(gui._selected_cell)
        self.assertIsNone(gui._highlight_rect)
        mock_ax.clear.assert_called()

    def test_board_gui_close_cleanup(self):
        """close() disconnects events and closes the figure."""
        from unittest.mock import MagicMock
        from src.interaction.board_state import create_initial_board

        board = create_initial_board()
        mock_plt = MagicMock()

        mock_fig = MagicMock()
        mock_ax = MagicMock()
        mock_fig.canvas.mpl_connect = MagicMock(return_value=1)
        mock_fig.canvas.mpl_disconnect = MagicMock()
        mock_fig.canvas.manager.set_window_title = MagicMock()

        import queue
        from src.interaction.board_gui import BoardGUI
        gui = BoardGUI.__new__(BoardGUI)
        gui._plt = mock_plt
        gui._Rectangle = MagicMock()
        gui._command_queue = queue.Queue()
        gui._selected_cell = None
        gui._highlight_rect = None
        gui._hover_rect = None
        gui._board = board
        gui._window_open = True
        gui._cid_click = 1
        gui._cid_motion = 2
        gui._cid_close = 3
        gui._fig = mock_fig
        gui._ax = mock_ax
        gui._last_draw_time = 0.0

        gui.close()
        self.assertFalse(gui._window_open)
        self.assertIsNone(gui._cid_click)
        self.assertIsNone(gui._cid_close)
        mock_plt.close.assert_called_once_with(mock_fig)

    def test_gui_create_board_gui_integration(self):
        """gui.create_board_gui() creates and registers a BoardGUI."""
        from unittest.mock import MagicMock
        from src.interaction.board_state import create_initial_board

        board = create_initial_board()
        mock_plt = MagicMock()

        mock_fig = MagicMock()
        mock_ax = MagicMock()
        mock_fig.canvas.mpl_connect = MagicMock(return_value=1)
        mock_fig.canvas.mpl_disconnect = MagicMock()
        mock_fig.canvas.manager.set_window_title = MagicMock()
        mock_fig.canvas.draw_idle = MagicMock()

        import queue
        from src.interaction.board_gui import BoardGUI

        # Build a mock BoardGUI via __new__ (avoids opening real window)
        gui = BoardGUI.__new__(BoardGUI)
        gui._plt = mock_plt
        gui._Rectangle = MagicMock()
        gui._command_queue = queue.Queue()
        gui._selected_cell = None
        gui._highlight_rect = None
        gui._hover_rect = None
        gui._board = board
        gui._window_open = True
        gui._cid_click = 1
        gui._cid_motion = 2
        gui._cid_close = 3
        gui._fig = mock_fig
        gui._ax = mock_ax
        gui._last_draw_time = 0.0
        gui._draw(board)

        # Register it via gui module
        import src.interaction.gui as gui_module
        saved = gui_module._active_board_gui
        gui_module._active_board_gui = gui
        try:
            from src.interaction.gui import get_active_board_gui, poll_gui_command

            self.assertIsNotNone(gui)
            self.assertIs(get_active_board_gui(), gui)
            self.assertTrue(gui.is_open)

            c1 = MagicMock()
            c1.xdata = 0.0
            c1.ydata = 0.0
            gui._on_click(c1)

            c2 = MagicMock()
            c2.xdata = 0.0
            c2.ydata = 1.0
            gui._on_click(c2)

            cmd = poll_gui_command(board, input_func=lambda p: "quit")
            self.assertIsNotNone(cmd)
            self.assertEqual(cmd.command_type, "move")
            self.assertEqual(cmd.from_cell, "A1")
            self.assertEqual(cmd.to_cell, "A2")
        finally:
            gui_module._active_board_gui = saved

    def test_gui_poll_command_falls_back_to_text_when_no_board_active(self):
        """poll_gui_command uses text input when no BoardGUI is active."""
        from src.interaction.board_state import create_initial_board
        from src.interaction.gui import poll_gui_command
        import src.interaction.gui as gui_module

        board = create_initial_board()

        saved = gui_module._active_board_gui
        gui_module._active_board_gui = None
        try:
            cmd = poll_gui_command(board, input_func=lambda p: "A1 A2")
            self.assertIsNotNone(cmd)
            self.assertEqual(cmd.from_cell, "A1")
        finally:
            gui_module._active_board_gui = saved


if __name__ == "__main__":
    unittest.main()
