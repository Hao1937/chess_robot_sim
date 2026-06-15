"""真实 PyBullet 回归套件——一键判断机械臂控制/IK 是否回归。

运行：
    PYTHONPATH=. ./.venv/Scripts/python.exe scripts/verify_pybullet.py

期望输出：4 项检查全部 [PASS]，进程退出码 0；任一失败退出码 1。

四项检查（对应 2026-06-14 根因修复）：
  1. IK 确定性 + 朝向：solve_ik 是 (target,seed) 的纯函数；所有解吸盘竖直 (-zz>0.9)。
     —— 防回归「PyBullet IK 用实时姿态作种子、不验证朝向」导致的第二步位姿失控。
  2. 连续多步无累积：连跑多步走棋，每步 EE 误差不随步数增长。
     —— 防回归「IK 从 home_pose 播种 + teleport 只在首段」导致的跨段误差累积。
  3. 流式平滑：执行期间每 sim step 关节位移有界、空等步占比低。
     —— 防回归「逐点固定沉降」导致的 94% 空等、肉眼卡顿。
  4. 无瞬移跳变：执行中无单步关节大跳变（resetJointState teleport 已移除）。

DIRECT 模式运行（不开 GUI）。各指标含义见每项检查内注释。
"""
from __future__ import annotations

import math
import os
import sys

os.environ.pop("CHESS_ROBOT_PYBULLET_GUI", None)
os.environ["CHESS_ROBOT_BOARD_GUI"] = "0"

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from src.common.config import DEFAULT_CONFIG as C
from src.interaction.board_state import create_initial_board
from src.interaction.cli import parse_command
from src.planning.chessboard_mapping import cell_to_world
from src.planning.ik_solver import solve_ik
from src.control.fk_solver import _get_tool0_z_axis
from src.simulation.load_robot import load_robot
from src.simulation.scene_builder import build_scene
from src.simulation._runtime import RUNTIME, p
import src.control.controller as ctrl
import main

# ── 阈值 ──
EE_ERR_MEAN_MAX = 0.02      # 每步 EE 平均误差上限 (m)
EE_ERR_PEAK_MAX = 0.06      # 每步 EE 峰值误差上限 (m，允许边缘列瞬态)
ZZ_MIN = 0.9                # 吸盘竖直度下限 (-zz)
IDLE_RATIO_MAX = 0.30       # 空等步占比上限
PER_STEP_DISP_MAX = 0.05    # 单 sim step 最大关节位移 (rad，超过即视为瞬移跳变)
MOVES = ["A4 A5", "C4 C5", "G4 G5", "B3 B7"]

_results: list[tuple[str, bool, str]] = []


def check(name: str, ok: bool, detail: str) -> None:
    _results.append((name, ok, detail))
    tag = "[PASS]" if ok else "[FAIL]"
    print(f"{tag} {name}: {detail}")


def main_run() -> int:
    if p is None:
        print("PyBullet 不可用——本套件需要真实 PyBullet。")
        return 1

    board = create_initial_board()
    robot = load_robot()
    scene = build_scene(config=C, obstacle_mode="mode_1")

    # ── 检查 1：IK 确定性 + 朝向 ──
    # 测可靠工作区（第 1-9 行）。第 10 行离基座最远(~0.79m，接近 0.9m 极限)，
    # 手臂伸到极限无法完全竖直(-zz≈0.67-0.85)，是 UR5 摆位的固有工作空间边界，
    # 非本次回归目标，故不纳入朝向断言。
    det_ok = True
    zz_min_all = 1.0
    seed = C.home_pose[:6]
    for cell in ["A1", "C4", "E5", "G7", "I9"]:
        t = cell_to_world(cell, C)
        tgt = (t[0], t[1], C.z_grasp)
        r1 = solve_ik(tgt, C, seed=seed)
        r2 = solve_ik(tgt, C, seed=seed)
        if r1 != r2:
            det_ok = False
        zz_min_all = min(zz_min_all, -_get_tool0_z_axis(r1, C)[2])
    check("1.IK确定性+朝向",
          det_ok and zz_min_all > ZZ_MIN,
          f"确定性={'一致' if det_ok else '不一致'}, 最小竖直度 -zz={zz_min_all:.3f} (阈值>{ZZ_MIN})")

    # ── 准备：包装 stepSimulation 记录每步运动 ──
    motion: list[float] = []
    prev = [None]
    _origstep = ctrl.p.stepSimulation

    def wstep(*a, **k):
        r = _origstep(*a, **k)
        cur = tuple(p.getJointState(RUNTIME.robot_id, j)[0] for j in RUNTIME.joint_indices)
        if prev[0] is not None:
            motion.append(max(abs(cur[k2] - prev[0][k2]) for k2 in range(6)))
        prev[0] = cur
        return r

    ctrl.p.stepSimulation = wstep

    # ── 检查 2/3/4：连续多步执行 ──
    per_move_mean: list[float] = []
    per_move_peak: list[float] = []
    move_zz_min = 1.0
    all_idle_ratio: list[float] = []
    all_peak_disp = 0.0

    for mv in MOVES:
        motion.clear()
        prev[0] = None
        res = main.run_command(parse_command(mv), board, scene, robot, C)
        ex = res["execution"]
        ee = ex.end_effector_errors
        per_move_mean.append(sum(ee) / len(ee))
        per_move_peak.append(max(ee))
        move_zz_min = min(move_zz_min, min(-_get_tool0_z_axis(w, C)[2] for w in ex.desired_joint_angles))
        if motion:
            idle = sum(1 for m in motion if m < 1e-4) / len(motion)
            all_idle_ratio.append(idle)
            all_peak_disp = max(all_peak_disp, max(motion))

    ctrl.p.stepSimulation = _origstep

    # 检查 2：连续多步无累积
    mean_ok = all(m < EE_ERR_MEAN_MAX for m in per_move_mean)
    peak_ok = all(pk < EE_ERR_PEAK_MAX for pk in per_move_peak)
    check("2.连续多步无累积",
          mean_ok and peak_ok and move_zz_min > ZZ_MIN,
          f"各步均值={[round(m,4) for m in per_move_mean]} 峰值={[round(m,4) for m in per_move_peak]} "
          f"竖直度 -zz_min={move_zz_min:.3f}")

    # 检查 3：流式平滑（空等步占比低）
    max_idle = max(all_idle_ratio) if all_idle_ratio else 1.0
    check("3.流式平滑无空等",
          max_idle < IDLE_RATIO_MAX,
          f"各步空等占比={[round(r*100) for r in all_idle_ratio]}% (阈值<{int(IDLE_RATIO_MAX*100)}%)")

    # 检查 4：无瞬移跳变
    check("4.无瞬移跳变",
          all_peak_disp < PER_STEP_DISP_MAX,
          f"单 sim step 最大关节位移={all_peak_disp:.4f} rad (阈值<{PER_STEP_DISP_MAX})")

    # ── 汇总 ──
    passed = sum(1 for _, ok, _ in _results if ok)
    total = len(_results)
    print("-" * 60)
    print(f"结果：{passed}/{total} 项通过")
    return 0 if passed == total else 1


if __name__ == "__main__":
    sys.exit(main_run())
