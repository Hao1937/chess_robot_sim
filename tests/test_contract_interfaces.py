import sys
import unittest
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT))


def chinese_rook_move() -> str:
    return chr(0x8f66) + chr(0x4e8c) + chr(0x5e73) + chr(0x4e03)


def chinese_cannon_move() -> str:
    return chr(0x70ae) + chr(0x4e94) + chr(0x8fdb) + chr(0x56db)


def chinese_front_rook_move() -> str:
    return chr(0x524d) + chinese_rook_move()


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
        self.assertGreaterEqual(len(scene.obstacles), 2)
        self.assertTrue(all(obstacle.obstacle_id.startswith("preset_column_") for obstacle in scene.obstacles))

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

        self.assertEqual(cells_for("mode_1"), [(4, 5)])
        self.assertEqual(cells_for("mode_2"), [(2, 5), (5, 5)])
        self.assertEqual(cells_for("mode_3"), [(2, 5), (4, 6), (6, 5)])
        self.assertTrue(all(obstacle.radius == 0.05 for obstacle in mode_1))
        self.assertTrue(all(obstacle.radius == 0.025 for obstacle in mode_2))
        self.assertTrue(all(obstacle.radius == 0.045 for obstacle in mode_3))
        self.assertTrue(all(obstacle.height == 0.30 for obstacle in mode_1 + mode_2 + mode_3))

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
            )
        finally:
            main.set_human_safety_zone = original_toggle

        self.assertEqual(visual_calls, [True, False])

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


    # ── PyBullet 实物仿真测试 ──

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

            ee_state = p.getLinkState(RUNTIME.robot_id, robot.end_effector_id, physicsClientId=client_id)
            ee_z_before = ee_state[0][2]
            self.assertLess(abs(ee_z_before - config.z_grasp), 0.03,
                            f"EE 应接近 grasp 高度，实际 z={ee_z_before:.4f}")

            # ── 吸附棋子 ──
            from src.simulation.attachment import attach_piece, detach_piece, _get_all_piece_body_ids

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

            self.assertLess(piece_error, 0.025,
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
            self.assertLess(transfer_error, 0.025,
                            f"棋子应跟随吸盘尖端: pad_tip=({pad_tip_transfer[0]:.4f},{pad_tip_transfer[1]:.4f},{pad_tip_transfer[2]:.4f}), "
                            f"expected piece=({expected_piece_transfer[0]:.4f},{expected_piece_transfer[1]:.4f},{expected_piece_transfer[2]:.4f}), "
                            f"actual piece=({piece_transfer[0]:.4f},{piece_transfer[1]:.4f},{piece_transfer[2]:.4f}), error={transfer_error:.4f}m")

            # ── 释放棋子 ──
            detach_result = detach_piece(piece_id=piece_id)
            self.assertTrue(detach_result.success, f"detach 失败: {detach_result.message}")

            # 约束应已移除
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


if __name__ == "__main__":
    unittest.main()
