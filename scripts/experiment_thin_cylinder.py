"""薄圆柱避障实验 v2 — 详细诊断版。

目标：找出为什么细圆柱 (r=0.012m @ C5) 没触发绕行。
"""
from __future__ import annotations
import os, sys, math
os.environ.pop("CHESS_ROBOT_PYBULLET_GUI", None)
os.environ["CHESS_ROBOT_BOARD_GUI"] = "0"
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from src.common.config import DEFAULT_CONFIG as C
from src.common.types import Obstacle, ObstacleShape
from src.planning.chessboard_mapping import cell_to_world
from src.planning.trajectory_planner import plan_trajectory
from src.planning.collision_checker import direct_path_clear, check_segment_collision
from src.control.fk_solver import _solve_fk_urdf_chain
from src.simulation.load_robot import load_robot
from src.simulation._runtime import p
from src.interaction.board_state import create_initial_board
from src.interaction.cli import parse_command
from src.planning.motion_primitives import build_motion_primitives
from src.planning.obstacle_map import build_primitive_obstacle_contexts
import main


def experiment():
    robot = load_robot()
    board = create_initial_board()

    # ── 障碍物：细圆柱浮空在 C5 ──
    c5 = cell_to_world("C5", C)
    thin_cylinder = Obstacle(
        obstacle_id="thin_rod",
        center_xyz=(c5[0], c5[1], C.z_board + 0.06),
        radius=0.012,
        height=0.22,
        dynamic=False,
        shape=ObstacleShape.VERTICAL_CYLINDER,
    )
    print(f"Config: z_board={C.z_board:.3f}, safety_margin={C.safety_margin:.4f}, "
          f"path_collision_check_step={C.path_collision_check_step:.4f}, "
          f"path_grid_resolution={C.path_grid_resolution:.4f}")
    print(f"Obstacle thin_rod @ C5: center_xyz=({c5[0]:.4f},{c5[1]:.4f},{C.z_board+0.06:.3f}) r={thin_cylinder.radius:.3f} h={thin_cylinder.height:.3f}")

    # ── 走法：C1 → C9 ──
    cmd = parse_command("C1 C9")
    actions = main.make_logical_actions(board, cmd)
    primitives = build_motion_primitives(actions, C)

    print(f"\nPrimitives ({len(primitives)}):")
    for i, p in enumerate(primitives):
        print(f"  [{i}] {p.primitive_type:10s} cell={p.cell or '-'} target_xyz=({p.target_xyz[0]:.4f},{p.target_xyz[1]:.4f},{p.target_xyz[2]:.4f})")

    contexts = build_primitive_obstacle_contexts(
        actions=actions, primitives=primitives, board=board,
        extra_obstacles=[thin_cylinder], human_hand_present=False, config=C,
    )

    # ── 逐个 primitive 诊断 ──
    for i, ctx in enumerate(contexts):
        prim = ctx.primitive
        print(f"\n--- Primitive [{i}] {prim.primitive_type} → {prim.target_xyz} ---")
        print(f"  Obstacles in context: {len(ctx.obstacles)}")
        thin_found = any(o.obstacle_id == "thin_rod" for o in ctx.obstacles)
        print(f"  thin_rod in obstacles: {thin_found}")

        if prim.primitive_type in ("approach", "transfer"):
            # 前一 waypoint 是上一段终点，初始为起点的上方
            c1_xyz = cell_to_world("C1", C)
            prev_xy = (c1_xyz[0], c1_xyz[1])
            start_xy = prev_xy
            end_xy = (prim.target_xyz[0], prim.target_xyz[1])
            z_plane = prim.target_xyz[2]

            print(f"  z_plane={z_plane:.4f}  start_xy={start_xy}  end_xy={end_xy}")

            # 直接调 collision checker
            clear = direct_path_clear(
                start_xy, end_xy, z_plane, ctx.obstacles,
                step_size=C.path_collision_check_step,
                safety_margin=C.safety_margin,
            )
            print(f"  direct_path_clear: {clear}")

            # 详细检测：逐点采样
            result = check_segment_collision(
                (start_xy[0], start_xy[1], z_plane),
                (end_xy[0], end_xy[1], z_plane),
                ctx.obstacles,
                step_size=C.path_collision_check_step,
                safety_margin=C.safety_margin,
            )
            print(f"  collision_free={result.collision_free}  min_clearance={result.min_clearance:.5f}  collision_point={result.collision_point}")

            # 单独检测 thin_rod
            for o in ctx.obstacles:
                if o.obstacle_id == "thin_rod":
                    ox, oy = o.center_xyz[0], o.center_xyz[1]
                    # 直线中点
                    mx, my = (start_xy[0]+end_xy[0])/2, (start_xy[1]+end_xy[1])/2
                    d_mid = math.hypot(mx-ox, my-oy)
                    clear_mid = d_mid - o.radius
                    print(f"  [thin_rod] 直线中点距障碍: {d_mid:.5f}m (clearance={clear_mid:.5f}, margin={C.safety_margin:.4f})")
                    print(f"  [thin_rod] blocked判定: {clear_mid <= C.safety_margin}")

    # ── 规划轨迹 ──
    print(f"\n=== Planning trajectory ===")
    traj = plan_trajectory(contexts, config=C)

    # 分析
    ee_positions = []
    for jw in traj.joint_waypoints:
        pos = _solve_fk_urdf_chain(jw, C)
        ee_positions.append(pos)

    start_xy = (ee_positions[0][0], ee_positions[0][1])
    end_xy = (ee_positions[-1][0], ee_positions[-1][1])
    obs_xy = (c5[0], c5[1])

    min_dist = min(math.hypot(p[0]-obs_xy[0], p[1]-obs_xy[1]) for p in ee_positions)
    dx = end_xy[0] - start_xy[0]; dy = end_xy[1] - start_xy[1]
    line_len = math.hypot(dx, dy)
    max_dev = max(abs((p[0]-start_xy[0])*dy - (p[1]-start_xy[1])*dx) / line_len if line_len>1e-9 else 0
                  for p in ee_positions)

    print(f"\n=== Result ===")
    print(f"  Waypoints: {len(traj.joint_waypoints)}")
    print(f"  EE start: ({ee_positions[0][0]:.4f},{ee_positions[0][1]:.4f},{ee_positions[0][2]:.4f})")
    print(f"  EE end:   ({ee_positions[-1][0]:.4f},{ee_positions[-1][1]:.4f},{ee_positions[-1][2]:.4f})")
    print(f"  Min dist to C5 obstacle: {min_dist:.5f}m")
    print(f"  Max lateral deviation:   {max_dev:.5f}m")
    print(f"  Obstacle radius: {thin_cylinder.radius:.4f}m")
    print(f"  Effective detour: {'YES' if max_dev > 0.015 else 'NO (essentially straight line)'}")

    # 速度模式分布
    fast_count = traj.speed_profile.count("fast")
    safe_count = traj.speed_profile.count("safe")
    print(f"  Speed: fast={fast_count} safe={safe_count}")

    print("\nDone.")


if __name__ == "__main__":
    if p is None:
        print("PyBullet not available.")
        sys.exit(1)
    experiment()
