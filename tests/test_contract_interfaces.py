import sys
import unittest
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT))


class ContractInterfaceTests(unittest.TestCase):
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
            joint_waypoints=[(0.0, 0.1, 0.2), (0.1, 0.2, 0.3)],
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

    def test_human_hand_obstacle_can_pause_when_target_blocked(self):
        from src.common.config import DEFAULT_CONFIG
        from src.common.types import Obstacle
        from src.planning.obstacle_map import assess_obstacle_intervention

        target_xyz = (
            DEFAULT_CONFIG.board_origin[0] + 4 * DEFAULT_CONFIG.cell_size,
            DEFAULT_CONFIG.board_origin[1] + 4 * DEFAULT_CONFIG.cell_size,
            DEFAULT_CONFIG.z_board,
        )
        blocking_hand = Obstacle(
            obstacle_id="human_hand_zone",
            center_xyz=target_xyz,
            radius=0.08,
            height=0.12,
            dynamic=True,
        )

        decision = assess_obstacle_intervention(target_xyz, [blocking_hand], DEFAULT_CONFIG)

        self.assertEqual(decision.status, "pause")
        self.assertIn("blocked", decision.reason)

    def test_main_demo_accepts_obstacle_mode_without_piece_attachment(self):
        from main import run_demo

        result = run_demo("obstacle_mode 2")

        self.assertEqual(result["command"].command_type, "obstacle_mode")
        self.assertGreaterEqual(result["trajectory_points"], 1)
        self.assertTrue(result["execution"].success)

    def test_red_chinese_notation_rook_horizontal_move(self):
        from src.common.types import BoardState, Piece, PieceColor, PieceType
        from src.interaction.chinese_notation import parse_chinese_move

        board = BoardState(
            pieces={
                "B1": Piece(piece_id="red_rook_2", kind=PieceType.ROOK, color=PieceColor.RED, cell="B1"),
            }
        )

        command = parse_chinese_move("车二平七", board)

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

        command = parse_chinese_move("炮五进四", board)

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

        with self.assertRaisesRegex(ValueError, "前/后"):
            parse_chinese_move("车二平七", board)

        command = parse_chinese_move("前车二平七", board)
        self.assertEqual(command.from_cell, "B4")
        self.assertEqual(command.to_cell, "G4")


if __name__ == "__main__":
    unittest.main()
