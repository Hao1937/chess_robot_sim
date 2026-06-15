"""调研避障失效根因（DIRECT 模式）。仅诊断，不修改。"""
from __future__ import annotations
import os, math
os.environ.pop("CHESS_ROBOT_PYBULLET_GUI", None)
os.environ["CHESS_ROBOT_BOARD_GUI"] = "0"
import sys
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from src.common.config import DEFAULT_CONFIG as C
from src.interaction.board_state import create_initial_board, make_logical_actions
from src.interaction.cli import parse_command
from src.interaction.chess_rules import validate_move
from src.planning.chessboard_mapping import cell_to_world
from src.planning.motion_primitives import build_motion_primitives
from src.planning.obstacle_map import build_primitive_obstacle_contexts
from src.planning.trajectory_planner import plan_trajectory
from src.planning.collision_checker import check_segment_collision_multi_z, direct_path_clear
from src.planning.ik_solver import solve_ik, is_reachable
from src.control.fk_solver import _solve_fk_urdf_chain, _get_tool0_z_axis
from src.simulation.load_robot import load_robot
from src.simulation.scene_builder import build_scene
from src.simulation._runtime import RUNTIME, p
import main

board = create_initial_board()
robot = load_robot()
scene = build_scene(config=C, obstacle_mode="mode_1")
print("障碍(mode_1):", [(o.obstacle_id, tuple(round(x,3) for x in o.center_xyz), 'r=%.3f'%o.radius, 'h=%.2f'%o.height) for o in scene.obstacles])

def diag_move(mv):
    print("="*70); print("命令:", mv)
    cmd = parse_command(mv)
    # 该命令是否需要在场上摆个棋子？E10/E9/E3 初始无子，先在起点放一个红兵做搬运
    # 用已有初始棋子的格子更真实：改用实际有子的走法见下方调用
    val = validate_move(board, cmd)
    if not val.is_legal:
        print("  非法走法(无子或违规):", val.reason); return
    actions = make_logical_actions(board, cmd)
    prims = build_motion_primitives(actions, C)
    ctxs = build_primitive_obstacle_contexts(actions=actions, primitives=prims, board=board,
                                             extra_obstacles=scene.obstacles, human_hand_present=False, config=C)
    # 逐 primitive：直线是否被挡 / 安全决策
    for ctx in ctxs:
        pr = ctx.primitive
        prev = None  # 仅看水平段是否穿障
        print(f"  prim {pr.primitive_type:9s} target={tuple(round(x,3) for x in pr.target_xyz)} "
              f"safety={ctx.safety_decision.status}({ctx.safety_decision.reason[:30]})")
    traj = plan_trajectory(ctxs, config=C)
    wps = traj.joint_waypoints
    # IK 可达性 + 朝向 + 位置误差
    bad_ik = 0; bad_zz = 0
    for i, (cart, jw) in enumerate(zip([cell_to_world(pr.cell, C) for pr in prims], wps[:0])):
        pass
    # 用 desired joint 反算 EE，与规划意图对比无直接 cart；改测每个 wp 的 -zz 和是否落在障碍上
    for jw in wps:
        ee = _solve_fk_urdf_chain(jw, C); zz = -_get_tool0_z_axis(jw, C)[2]
        if zz < 0.85: bad_zz += 1
        for o in scene.obstacles:
            d = math.hypot(ee[0]-o.center_xyz[0], ee[1]-o.center_xyz[1])
            if d < o.radius and ee[2] < o.height:  # EE 落在障碍柱内
                bad_ik += 1; break
    print(f"  轨迹 {len(wps)} wp | 朝向差(-zz<0.85)={bad_zz} | EE落在障碍内={bad_ik}")
    # 执行
    res = main.run_command(cmd, board, scene, robot, C)
    ex = res["execution"]; ee = ex.end_effector_errors
    print(f"  执行 ee_err max={max(ee):.4f} mean={sum(ee)/len(ee):.4f} | safety={[s.status for s in res['safety_decisions']]}")

# 用初始有子的走法测试（红兵在第4行：A4 C4 E4 G4 I4；炮在B3 H3；车马象士将在第1行）
# E4->E6 会撞上 E6 障碍（descend 落点就是障碍）；E4->E8 跨过障碍需绕行
for mv in ["E10 E9", "E4 E5", "E4 E6", "A4 I4", "E4 E8"]:
    try:
        diag_move(mv)
    except Exception as exc:
        import traceback; print("  异常:", exc); traceback.print_exc()
print("DONE")
