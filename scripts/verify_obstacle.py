"""避障/可达性验收套件——定义「完成」标准（真实 PyBullet DIRECT）。

配套计划：docs/superpowers/plans/2026-06-15-obstacle-avoidance-plan.md
运行：PYTHONPATH=. ./.venv/Scripts/python.exe scripts/verify_obstacle.py
期望：实施完成后 6 项全 [PASS]、退出码 0；实施前部分 [FAIL] 属正常（红→绿目标）。

实施 agent 调整障碍位置/演示走法时，请同步更新下方 DEMO_CASES 与各检查的常量，
断言逻辑（定义"完成"）保持不变。
"""
from __future__ import annotations
import os, math, sys
os.environ.pop("CHESS_ROBOT_PYBULLET_GUI", None)
os.environ["CHESS_ROBOT_BOARD_GUI"] = "0"
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from src.common.config import DEFAULT_CONFIG as C
from src.common.types import MotionPrimitive, Obstacle
from src.planning.chessboard_mapping import cell_to_world
from src.planning.trajectory_planner import plan_trajectory
from src.planning.ik_solver import solve_ik
from src.control.fk_solver import _solve_fk_urdf_chain, _get_tool0_z_axis
from src.simulation.load_robot import load_robot
from src.simulation.scene_builder import build_scene
from src.simulation._runtime import RUNTIME, p
from src.interaction.board_state import create_initial_board
from src.interaction.cli import parse_command
import main

# ── 阈值 ──
ZZ_MIN = 0.85  # 与 feasibility_zz_min 一致（计划建议 0.85）
POS_TOL = 0.01
EE_ERR_MAX = 0.03  # 控制器有~0.02m 噪点，1cm 太严格
OVERFLY_CLEAR = 0.03   # 越顶后 EE 应高出障碍顶至少此值

# 演示走法表（实施 agent 随障碍重设计同步更新）
# (obstacle_mode, 走法, 期望: 'success' | 'reject')
DEMO_CASES = [
    ("mode_1", "C1 C9", "success"),   # 侧绕：车沿C列穿越mode_1高细柱(C6)
    ("mode_2", "E1 E5", "success"),   # 越顶：沿E列飞越mode_2矮粗柱(E5)
    ("mode_3", "B1 B9", "success"),   # 绕行：车沿B列在mode_3闸口(B6+E6)间穿行
]

_results = []
_ROBOT = None  # 全进程仅 load_robot 一次（多次重载会创建多机器人并使 FK 缓存失效）


def _robot():
    global _ROBOT
    if _ROBOT is None:
        _ROBOT = load_robot()
    return _ROBOT


def check(name, ok, detail):
    _results.append(ok)
    print(f"{'[PASS]' if ok else '[FAIL]'} {name}: {detail}")


def safe(name, fn):
    try:
        fn()
    except Exception as exc:
        check(name, False, f"异常/未实现: {type(exc).__name__}: {exc}")


# ── 检查 1：base 可达性（全 90 格 + 吃子区）──
def check_base():
    _robot(); build_scene(config=C, obstacle_mode="none")
    bad = []
    cells = [f"{c}{r}" for c in "ABCDEFGHI" for r in range(1, 11)]
    cells += [f"CAPTURED_BLACK_{i}" for i in range(1, 6)] + [f"CAPTURED_RED_{i}" for i in range(1, 6)]
    worst_zz = 1.0
    for cell in cells:
        try:
            t = cell_to_world(cell, C)
        except Exception:
            continue
        tgt = (t[0], t[1], C.z_grasp)
        j = solve_ik(tgt, C, seed=C.home_pose[:6])
        ee = _solve_fk_urdf_chain(j, C); zz = -_get_tool0_z_axis(j, C)[2]
        worst_zz = min(worst_zz, zz)
        if zz < ZZ_MIN or math.dist(ee, tgt) > POS_TOL:
            bad.append(cell)
    check("1.base可达性", len(bad) == 0,
          f"不良格={len(bad)} {bad[:6]} 最差-zz={worst_zz:.3f} (要求0不良)")


# ── 检查 2：E10→E9 修复 ──
def check_e10e9():
    board = create_initial_board(); robot = _robot()
    scene = build_scene(config=C, obstacle_mode="mode_1")
    cmd = parse_command("E10 E9")
    res = main.run_command(cmd, board, scene, robot, C)
    ee = res["execution"].end_effector_errors
    check("2.E10→E9修复", max(ee) < EE_ERR_MAX,
          f"ee_err max={max(ee):.4f} (要求<{EE_ERR_MAX})")


# ── 检查 3：2D 侧绕（planner 级，穿越高柱）──
def check_2d_detour():
    _robot(); build_scene(config=C, obstacle_mode="none")
    # 高柱挡在直线中点，2D 应侧绕
    obs = [Obstacle("t", (0.24, 0.30, 0.0), radius=0.05, height=0.34, dynamic=False)]
    start = (0.24, 0.12, 0.18); end = (0.24, 0.48, 0.18)
    prims = [MotionPrimitive("approach", "X", start, "fast", None),
             MotionPrimitive("transfer", "Y", end, "fast", None)]
    traj = plan_trajectory(prims, obstacles=obs, config=C)
    pen = max((obs[0].radius - math.hypot(_solve_fk_urdf_chain(jw, C)[0]-0.24,
              _solve_fk_urdf_chain(jw, C)[1]-0.30)) for jw in traj.joint_waypoints
              if _solve_fk_urdf_chain(jw, C)[2] < obs[0].height)
    maxdev = max(abs(_solve_fk_urdf_chain(jw, C)[0]-0.24) for jw in traj.joint_waypoints)
    check("3.2D侧绕", pen < 0 and maxdev > 0.03,
          f"EE侵入={pen:.4f}(<0=避开) 侧移={maxdev:.3f}(>0.03=绕行)")


# ── 检查 4：3D 越顶（2D 被墙挡死时抬高越过）──
def check_3d_overfly():
    _robot(); build_scene(config=C, obstacle_mode="none")
    wall_h = 0.24
    # 一排宽柱横跨搜索边界形成"墙"，2D A* 绕不过 → 须越顶
    # 柱子半径 0.12 + safety_margin 0.015 = 0.135 膨胀，间距 0.24 保证重叠
    wall_x = [-0.48, -0.24, 0.0, 0.24, 0.48, 0.72, 0.96]
    wall = [Obstacle(f"w{i}", (x, 0.30, 0.0), radius=0.12, height=wall_h, dynamic=False)
            for i, x in enumerate(wall_x)]
    start = (0.24, 0.12, 0.18); end = (0.24, 0.48, 0.18)
    prims = [MotionPrimitive("approach", "X", start, "fast", None),
             MotionPrimitive("transfer", "Y", end, "fast", None)]
    traj = plan_trajectory(prims, obstacles=wall, config=C)
    max_z = max(_solve_fk_urdf_chain(jw, C)[2] for jw in traj.joint_waypoints)
    check("4.3D越顶", max_z > wall_h + OVERFLY_CLEAR,
          f"轨迹最高EE z={max_z:.3f} (要求>{wall_h+OVERFLY_CLEAR:.2f}=越过墙顶)")


# ── 检查 5：可行性闸门 + 拒绝 ──
def check_reject():
    from src.interaction.board_state import create_initial_board, make_logical_actions
    from src.planning.motion_primitives import build_motion_primitives
    from src.planning.obstacle_map import build_primitive_obstacle_contexts
    # 目标格正上方压一根障碍 → descend 落点不可达
    board = create_initial_board()
    # 找一个有子且可合法移动到被封格的走法较繁琐；此处直接测可行性函数
    from src.planning import feasibility  # 实施需提供该模块/函数
    actions = make_logical_actions(board, parse_command("A4 A5"))
    prims = build_motion_primitives(actions, C)
    blocking = [Obstacle("blk", cell_to_world("A5", C), radius=0.06, height=0.30, dynamic=False)]
    ctxs = build_primitive_obstacle_contexts(actions=actions, primitives=prims, board=board,
                                             extra_obstacles=blocking, human_hand_present=False, config=C)
    traj = plan_trajectory(ctxs, config=C)
    ok, reason = feasibility.validate_trajectory_feasibility(ctxs, traj, C)
    check("5.拒绝不可达", (ok is False) and ("不可达" in reason),
          f"可行={ok} 原因='{reason}'")


# ── 检查 6：无障碍回归 ──
def check_regression():
    board = create_initial_board(); robot = _robot()
    scene = build_scene(config=C, obstacle_mode="none")
    worst = 0.0
    for mv in ["A4 A5", "C4 C5", "G4 G5"]:
        res = main.run_command(parse_command(mv), board, scene, robot, C)
        worst = max(worst, max(res["execution"].end_effector_errors))
    check("6.无障碍回归", worst < EE_ERR_MAX, f"最差 ee_err={worst:.4f}")


if __name__ == "__main__":
    if p is None:
        print("PyBullet 不可用——本套件需要真实 PyBullet。"); sys.exit(1)
    for name, fn in [("1", check_base), ("2", check_e10e9), ("3", check_2d_detour),
                     ("4", check_3d_overfly), ("5", check_reject), ("6", check_regression)]:
        safe(name, fn)
    passed = sum(_results)
    print("-" * 60)
    print(f"结果：{passed}/{len(_results)} 项通过")
    sys.exit(0 if passed == len(_results) and len(_results) == 6 else 1)
