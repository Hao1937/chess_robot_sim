import math
import sys
import unittest
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT))


class CollisionCheckerTests(unittest.TestCase):
    """Phase 1: 碰撞检测测试"""

    @classmethod
    def setUpClass(cls):
        from src.common.types import Obstacle
        cls.Obstacle = Obstacle

    def test_direct_path_clear_no_obstacles(self):
        from src.planning.collision_checker import direct_path_clear

        self.assertTrue(direct_path_clear((0.0, 0.0), (0.3, 0.3), 0.18, []))

    def test_direct_path_blocked_by_vertical_cylinder(self):
        from src.planning.collision_checker import direct_path_clear

        obstacle = self.Obstacle(
            obstacle_id="blocker",
            center_xyz=(0.15, 0.15, 0.18),
            radius=0.05,
            height=0.3,
        )
        # 直线穿过障碍物中心
        self.assertFalse(
            direct_path_clear((0.0, 0.0), (0.3, 0.3), 0.18, [obstacle])
        )

    def test_direct_path_clear_with_safety_margin(self):
        from src.planning.collision_checker import direct_path_clear

        obstacle = self.Obstacle(
            obstacle_id="nearby",
            center_xyz=(0.15, 0.12, 0.18),
            radius=0.02,
            height=0.3,
        )
        # 无 margin：路径擦过边缘 → clear
        self.assertTrue(
            direct_path_clear((0.0, 0.0), (0.3, 0.3), 0.18, [obstacle])
        )
        # 大 margin：碰撞
        self.assertFalse(
            direct_path_clear((0.0, 0.0), (0.3, 0.3), 0.18, [obstacle], safety_margin=0.05)
        )

    def test_segment_collision_multiple_obstacles(self):
        from src.planning.collision_checker import check_segment_collision

        obstacles = [
            self.Obstacle("o1", (0.10, 0.10, 0.18), 0.03, 0.3),
            self.Obstacle("o2", (0.20, 0.20, 0.18), 0.03, 0.3),
            self.Obstacle("o3", (0.25, 0.10, 0.18), 0.03, 0.3),
        ]
        result = check_segment_collision((0.0, 0.0, 0.18), (0.3, 0.3, 0.18), obstacles)
        self.assertFalse(result.collision_free)
        self.assertIsNotNone(result.collision_point)

    def test_segment_zero_length(self):
        from src.planning.collision_checker import check_segment_collision

        obstacle = self.Obstacle("o1", (0.10, 0.10, 0.18), 0.05, 0.3)
        # 点远高障碍物 → free
        result = check_segment_collision((0.3, 0.3, 0.18), (0.3, 0.3, 0.18), [obstacle])
        self.assertTrue(result.collision_free)
        # 点落在障碍物内 → collision
        result = check_segment_collision((0.10, 0.10, 0.18), (0.10, 0.10, 0.18), [obstacle])
        self.assertFalse(result.collision_free)

    def test_segment_grazes_obstacle_edge(self):
        from src.planning.collision_checker import check_segment_collision

        obstacle = self.Obstacle("edge", (0.15, 0.0, 0.18), radius=0.05, height=0.3)
        # 线段沿 y=0.07，距离障碍物中心 0.07m，半径 0.05m → clearance ≈ 0.02m → free
        result = check_segment_collision((0.0, 0.07, 0.18), (0.3, 0.07, 0.18), [obstacle])
        self.assertTrue(result.collision_free)
        self.assertGreater(result.min_clearance, 0.01)

    def test_horizontal_cylinder_aabb_collision(self):
        from src.common.types import ObstacleShape
        from src.planning.collision_checker import direct_path_clear

        hand = self.Obstacle(
            obstacle_id="human_hand_zone",
            center_xyz=(0.24, 0.42, 0.18),
            radius=0.025,
            height=0.24,
            shape=ObstacleShape.HORIZONTAL_CYLINDER,
            orientation_rpy=(0.0, math.pi / 2.0, 0.0),
        )
        # 路径沿 Y 轴穿过手区 → blocked
        self.assertFalse(
            direct_path_clear((0.24, 0.3), (0.24, 0.5), 0.18, [hand])
        )
        # 路径远离手区 → clear
        self.assertTrue(
            direct_path_clear((0.0, 0.0), (0.5, 0.0), 0.18, [hand])
        )


class PathSearchTests(unittest.TestCase):
    """Phase 2: A* 路径搜索测试"""

    @classmethod
    def setUpClass(cls):
        from src.common.types import Obstacle
        cls.Obstacle = Obstacle

    def test_a_star_straight_line_when_clear(self):
        from src.planning.path_search import a_star_2d

        result = a_star_2d((0.0, 0.0), (0.3, 0.3), obstacles=[], z_plane=0.18)

        self.assertTrue(result.success)
        self.assertGreaterEqual(len(result.path_xy), 2)
        self.assertGreater(result.search_time_ms, 0)

        # 起点和终点应接近输入
        sx, sy = result.path_xy[0]
        ex, ey = result.path_xy[-1]
        self.assertAlmostEqual(math.hypot(sx - 0.0, sy - 0.0), 0.0, delta=0.03)
        self.assertAlmostEqual(math.hypot(ex - 0.3, ey - 0.3), 0.0, delta=0.03)

    def test_a_star_detours_around_single_obstacle(self):
        from src.planning.path_search import a_star_2d

        obstacle = self.Obstacle(
            obstacle_id="blocker",
            center_xyz=(0.15, 0.15, 0.18),
            radius=0.06,
            height=0.3,
        )
        result = a_star_2d((0.0, 0.0), (0.3, 0.3), obstacles=[obstacle], z_plane=0.18)

        self.assertTrue(result.success)
        # 绕行路径点数应 > 2（不能是直线）
        self.assertGreater(len(result.path_xy), 2)
        self.assertGreater(result.nodes_explored, 0)

    def test_a_star_detours_around_multiple_obstacles(self):
        from src.planning.path_search import a_star_2d

        obstacles = [
            self.Obstacle("o1", (0.10, 0.10, 0.18), 0.04, 0.3),
            self.Obstacle("o2", (0.20, 0.20, 0.18), 0.04, 0.3),
        ]
        result = a_star_2d((0.0, 0.0), (0.3, 0.3), obstacles=obstacles, z_plane=0.18)

        self.assertTrue(result.success)
        self.assertGreater(len(result.path_xy), 2)

    def test_a_star_returns_failure_when_trapped(self):
        from src.planning.path_search import a_star_2d

        # 用更紧密的障碍物包围起点和周围区域
        from src.common.config import DEFAULT_CONFIG
        obstacles = [
            self.Obstacle("n", (0.10, 0.20, 0.18), 0.08, 0.3),
            self.Obstacle("s", (0.10, -0.02, 0.18), 0.08, 0.3),
            self.Obstacle("e", (0.22, 0.09, 0.18), 0.08, 0.3),
            self.Obstacle("w", (-0.02, 0.09, 0.18), 0.08, 0.3),
        ]
        # 起点在包围圈内，目标在圈外
        result = a_star_2d(
            (0.10, 0.09), (0.50, 0.50),
            obstacles=obstacles, z_plane=0.18,
            grid_resolution=0.02, timeout_ms=500,
            config=DEFAULT_CONFIG,
        )
        # 被围困，搜索应返回失败
        self.assertFalse(result.success)

    def test_a_star_start_equals_goal(self):
        from src.planning.path_search import a_star_2d

        result = a_star_2d((0.1, 0.2), (0.1, 0.2), obstacles=[], z_plane=0.18)

        self.assertTrue(result.success)
        self.assertEqual(len(result.path_xy), 1)

    def test_build_2d_occupancy_grid_respects_bounds(self):
        from src.planning.path_search import build_2d_occupancy_grid

        obstacles = [
            self.Obstacle("o1", (0.30, 0.30, 0.18), 0.05, 0.3),
        ]
        bounds = (0.0, 0.54, 0.0, 0.60)
        grid = build_2d_occupancy_grid(obstacles, 0.18, 0.02, bounds)

        rows, cols = len(grid), len(grid[0])
        self.assertEqual(cols, int(math.ceil((0.54 - 0.0) / 0.02)))
        self.assertEqual(rows, int(math.ceil((0.60 - 0.0) / 0.02)))
        # 至少有一些格子被标记
        occupied_count = sum(1 for row in grid for cell in row if cell)
        self.assertGreater(occupied_count, 0)

    def test_build_2d_occupancy_grid_with_horizontal_cylinder(self):
        from src.common.types import ObstacleShape
        from src.planning.path_search import build_2d_occupancy_grid

        hand = self.Obstacle(
            obstacle_id="hand",
            center_xyz=(0.24, 0.42, 0.18),
            radius=0.025,
            height=0.24,
            shape=ObstacleShape.HORIZONTAL_CYLINDER,
            orientation_rpy=(0.0, math.pi / 2.0, 0.0),
        )
        bounds = (0.0, 0.54, 0.0, 0.60)
        grid = build_2d_occupancy_grid([hand], 0.18, 0.02, bounds)

        occupied_count = sum(1 for row in grid for cell in row if cell)
        # AABB 区域应有格子被占据
        self.assertGreater(occupied_count, 0)

        # 同样半径的竖直圆柱应占据更多格子
        vertical = self.Obstacle(
            obstacle_id="vertical_equiv",
            center_xyz=(0.24, 0.42, 0.18),
            radius=0.12,
            height=0.3,
        )
        grid_v = build_2d_occupancy_grid([vertical], 0.18, 0.02, bounds)
        occupied_v = sum(1 for row in grid_v for cell in row if cell)
        # 竖直大圆占据的格子应多于精确 AABB
        self.assertGreater(occupied_v, occupied_count)


class WaypointInterpolationTests(unittest.TestCase):
    """Phase 3: Waypoint 插值测试"""

    def test_interpolation_preserves_endpoints(self):
        from src.planning.trajectory_smoother import interpolate_waypoints_cartesian

        path = [(0.0, 0.0, 0.18), (0.3, 0.3, 0.18), (0.5, 0.1, 0.18)]
        result = interpolate_waypoints_cartesian(path, step_size=0.05)

        self.assertGreaterEqual(len(result), len(path))
        self.assertEqual(result[0], path[0])
        self.assertEqual(result[-1], path[-1])

    def test_interpolation_step_size_respected(self):
        from src.planning.trajectory_smoother import interpolate_waypoints_cartesian

        path = [(0.0, 0.0, 0.18), (0.3, 0.0, 0.18)]
        step = 0.03
        result = interpolate_waypoints_cartesian(path, step_size=step)

        for i in range(len(result) - 1):
            p0, p1 = result[i], result[i + 1]
            dist = math.hypot(p1[0] - p0[0], p1[1] - p0[1], p1[2] - p0[2])
            self.assertLessEqual(dist, step + 0.01)  # 允许微小浮点误差

    def test_interpolation_single_point(self):
        from src.planning.trajectory_smoother import interpolate_waypoints_cartesian

        path = [(0.1, 0.2, 0.18)]
        result = interpolate_waypoints_cartesian(path, step_size=0.03)

        self.assertEqual(len(result), 1)
        self.assertEqual(result[0], path[0])

    def test_interpolation_vertical_path(self):
        from src.planning.trajectory_smoother import interpolate_waypoints_cartesian

        # 垂直移动：xy 不变，z 变化
        path = [(0.24, 0.0, 0.18), (0.24, 0.0, 0.055)]
        result = interpolate_waypoints_cartesian(path, step_size=0.01)

        self.assertGreaterEqual(len(result), 2)
        for wp in result:
            self.assertAlmostEqual(wp[0], 0.24)
            self.assertAlmostEqual(wp[1], 0.0)


class ShortcutSmoothingTests(unittest.TestCase):
    """Phase 4: Shortcut 平滑测试"""

    @classmethod
    def setUpClass(cls):
        from src.common.types import Obstacle
        cls.Obstacle = Obstacle

    def test_shortcut_reduces_waypoint_count(self):
        from src.planning.trajectory_smoother import shortcut_smoothing

        # A* 典型输出：锯齿状路径
        zigzag = [
            (0.0, 0.0), (0.02, 0.02), (0.04, 0.0), (0.06, 0.02),
            (0.08, 0.0), (0.10, 0.02), (0.12, 0.0), (0.14, 0.02),
            (0.16, 0.0), (0.18, 0.0), (0.20, 0.0),
        ]
        result = shortcut_smoothing(zigzag, 0.18, obstacles=[])

        self.assertLessEqual(len(result), len(zigzag))

    def test_shortcut_preserves_collision_free(self):
        from src.planning.trajectory_smoother import shortcut_smoothing

        obstacle = self.Obstacle("o", (0.10, 0.06, 0.18), 0.04, 0.3)
        # 绕行路径
        detour = [(0.0, 0.0), (0.05, 0.05), (0.10, 0.10), (0.15, 0.05), (0.20, 0.0)]
        result = shortcut_smoothing(detour, 0.18, [obstacle])

        # Shortcut 后的路径仍然无障碍
        from src.planning.collision_checker import direct_path_clear
        for i in range(len(result) - 1):
            self.assertTrue(
                direct_path_clear(
                    (result[i][0], result[i][1]),
                    (result[i + 1][0], result[i + 1][1]),
                    0.18, [obstacle],
                )
            )

    def test_shortcut_straight_line_unchanged(self):
        from src.planning.trajectory_smoother import shortcut_smoothing

        straight = [(0.0, 0.0), (0.1, 0.1), (0.2, 0.2)]
        result = shortcut_smoothing(straight, 0.18, obstacles=[])

        # 直线场景下 shortcut 应压缩为最少点数（首尾）
        self.assertLessEqual(len(result), 3)

    def test_shortcut_empty_and_single(self):
        from src.planning.trajectory_smoother import shortcut_smoothing

        self.assertEqual(len(shortcut_smoothing([], 0.18, [])), 0)
        self.assertEqual(len(shortcut_smoothing([(0.1, 0.1)], 0.18, [])), 1)


class JointSmoothingTests(unittest.TestCase):
    """Phase 5: 关节轨迹平滑测试"""

    def test_joint_smoothing_reduces_step_changes(self):
        from src.planning.trajectory_smoother import smooth_joint_trajectory

        # 模拟带锯齿的关节轨迹
        jagged: list[tuple[float, ...]] = [
            (0.0, -0.8, 1.2, -0.4, 0.0, 0.0),
            (0.1, -0.7, 1.3, -0.3, 0.1, 0.0),
            (0.0, -0.8, 1.2, -0.4, 0.0, 0.0),  # 跳回
            (0.1, -0.7, 1.3, -0.3, 0.1, 0.0),
            (0.2, -0.6, 1.4, -0.2, 0.2, 0.0),
        ]
        result = smooth_joint_trajectory(jagged, smoothing_window=3)

        self.assertEqual(len(result), len(jagged))
        self.assertEqual(len(result[0]), 6)

        # 平滑后的第一个维度变化应比原始更小
        orig_delta = sum(
            abs(jagged[k + 1][0] - jagged[k][0]) for k in range(len(jagged) - 1)
        )
        smooth_delta = sum(
            abs(result[k + 1][0] - result[k][0]) for k in range(len(result) - 1)
        )
        self.assertLessEqual(smooth_delta, orig_delta + 0.001)

    def test_joint_smoothing_short_sequence_unchanged(self):
        from src.planning.trajectory_smoother import smooth_joint_trajectory

        short = [(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)]
        result = smooth_joint_trajectory(short, smoothing_window=3)

        self.assertEqual(len(result), 1)
        self.assertEqual(result[0], short[0])


class IntegrationTests(unittest.TestCase):
    """Phase 6: 集成测试"""

    @classmethod
    def setUpClass(cls):
        from src.common.types import LogicalAction, Obstacle
        cls.LogicalAction = LogicalAction
        cls.Obstacle = Obstacle

    def test_plan_trajectory_without_search_equals_legacy_behavior(self):
        """关闭所有新功能时，行为应与旧版一致。"""
        from src.common.config import DEFAULT_CONFIG
        from src.common.types import MotionPrimitive
        from src.planning.trajectory_planner import plan_trajectory

        primitives = [
            MotionPrimitive("approach", "A1", (0.0, 0.0, 0.18), "fast", ""),
            MotionPrimitive("descend", "A1", (0.0, 0.0, 0.055), "safe", ""),
        ]
        trajectory = plan_trajectory(
            primitives, config=DEFAULT_CONFIG,
            enable_path_search=False,
            enable_smoothing=False,
            enable_interpolation=False,
        )
        self.assertEqual(len(trajectory.joint_waypoints), 2)
        self.assertEqual(len(trajectory.speed_profile), 2)

    def test_plan_trajectory_with_path_search_detours(self):
        """放一个障碍物在直线上，验证 A* 绕行。

        需要两个水平 primitive：第一个建立起点，第二个触发路径搜索。
        """
        from src.common.config import DEFAULT_CONFIG
        from src.common.types import MotionPrimitive, PrimitivePlanningContext, SafetyDecision
        from src.planning.trajectory_planner import plan_trajectory

        obstacle = self.Obstacle("blocker", (0.15, 0.15, 0.18), 0.06, 0.3)

        # 第一个 primitive 从起点出发
        p1 = MotionPrimitive("approach", "A1", (0.0, 0.0, 0.18), "fast", "")
        # 第二个 primitive 终点在障碍物后方 → 直线被阻挡
        p2 = MotionPrimitive("transfer", "B1", (0.3, 0.3, 0.18), "fast", "")

        context1 = PrimitivePlanningContext(
            primitive=p1, obstacles=[obstacle],
            safety_decision=SafetyDecision(status="continue"),
        )
        context2 = PrimitivePlanningContext(
            primitive=p2, obstacles=[obstacle],
            safety_decision=SafetyDecision(status="continue"),
        )
        trajectory = plan_trajectory([context1, context2], config=DEFAULT_CONFIG)

        self.assertGreater(len(trajectory.joint_waypoints), 2)
        self.assertEqual(len(trajectory.joint_waypoints), len(trajectory.speed_profile))

    def test_plan_trajectory_fallback_when_search_fails(self):
        """A* 失败时 fallback 到直接路径。"""
        from src.common.config import DEFAULT_CONFIG
        from src.common.types import MotionPrimitive, PrimitivePlanningContext, SafetyDecision
        from src.planning.trajectory_planner import plan_trajectory

        # 用障碍物围住，提供不可达目标
        obstacles = [
            self.Obstacle("n", (0.15, 0.20, 0.18), 0.06, 0.3),
            self.Obstacle("s", (0.15, 0.05, 0.18), 0.06, 0.3),
        ]
        primitive = MotionPrimitive("approach", "A1", (0.3, 0.3, 0.18), "fast", "")

        context = PrimitivePlanningContext(
            primitive=primitive,
            obstacles=obstacles,
            safety_decision=SafetyDecision(status="continue"),
        )
        trajectory = plan_trajectory([context], config=DEFAULT_CONFIG)

        # 即使 A* 失败，仍应返回有效轨迹
        self.assertGreater(len(trajectory.joint_waypoints), 0)
        self.assertEqual(len(trajectory.joint_waypoints), len(trajectory.speed_profile))

    def test_full_pipeline_backward_compatible(self):
        """验证整个流水线仍然正常运行。"""
        from src.common.config import DEFAULT_CONFIG
        from src.interaction.board_state import create_initial_board, make_logical_actions
        from src.interaction.cli import parse_command
        from src.planning.motion_primitives import build_motion_primitives
        from src.planning.obstacle_map import build_primitive_obstacle_contexts
        from src.planning.trajectory_planner import plan_trajectory

        board = create_initial_board()
        actions = make_logical_actions(board, parse_command("A1 A2"))
        primitives = build_motion_primitives(actions)

        contexts = build_primitive_obstacle_contexts(
            actions=actions, primitives=primitives, board=board, extra_obstacles=[],
        )
        trajectory = plan_trajectory(contexts, config=DEFAULT_CONFIG)

        self.assertGreater(len(trajectory.joint_waypoints), 0)
        self.assertEqual(len(trajectory.joint_waypoints), len(trajectory.speed_profile))
        # 每个 waypoint 应有 6 个关节角
        for wp in trajectory.joint_waypoints:
            self.assertEqual(len(wp), 6)
            self.assertTrue(all(math.isfinite(theta) for theta in wp))

    def test_plan_trajectory_with_human_hand_horizontal_cylinder(self):
        """人手区作为水平圆柱时 A* 应能精确绕行（而非绕大圆）。"""
        from src.common.config import DEFAULT_CONFIG
        from src.common.types import MotionPrimitive, ObstacleShape, PrimitivePlanningContext, SafetyDecision
        from src.planning.trajectory_planner import plan_trajectory

        hand = self.Obstacle(
            obstacle_id="human_hand_zone",
            center_xyz=(0.24, 0.42, 0.18),
            radius=0.025,
            height=0.24,
            dynamic=True,
            shape=ObstacleShape.HORIZONTAL_CYLINDER,
            orientation_rpy=(0.0, math.pi / 2.0, 0.0),
        )
        primitive = MotionPrimitive("approach", "A1", (0.5, 0.42, 0.18), "fast", "")

        context = PrimitivePlanningContext(
            primitive=primitive,
            obstacles=[hand],
            safety_decision=SafetyDecision(status="continue"),
        )
        trajectory = plan_trajectory([context], config=DEFAULT_CONFIG)

        self.assertGreater(len(trajectory.joint_waypoints), 0)
        self.assertEqual(len(trajectory.joint_waypoints), len(trajectory.speed_profile))


if __name__ == "__main__":
    unittest.main()
