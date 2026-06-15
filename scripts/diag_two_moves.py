"""深入诊断：连续走棋的逐步 IK / 执行指标 dump（DIRECT 模式）。

用途：当 verify_pybullet.py 某项失败时，用本脚本定位是哪一步、哪个 waypoint 出问题。
与 verify_pybullet.py 的区别：本脚本只 dump 指标不做断言，输出更细。

运行：
    PYTHONPATH=. ./.venv/Scripts/python.exe scripts/diag_two_moves.py

每条命令输出：
    规划前物理关节         —— 机器人当前姿态（IK 链种子来源）
    IK 朝向不竖直数        —— -zz<0.9 的 waypoint 数（应为 0，除第 10 行边缘）
    IK 位置误差>2cm 数     —— IK 未收敛的 waypoint 数（应为 0）
    -zz 范围              —— 吸盘竖直度区间（越接近 1 越竖直）
    执行 joint/ee 误差     —— 实际跟踪误差（应接近 0，证明无卡顿/累积）
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
from src.control.fk_solver import _get_tool0_z_axis
from src.simulation.load_robot import load_robot
from src.simulation.scene_builder import build_scene
from src.simulation._runtime import RUNTIME, p
import main

MOVES = ["A4 A5", "C4 C5", "E4 E5", "G4 G5"]


def phys():
    if RUNTIME.robot_id is None:
        return None
    return tuple(round(p.getJointState(RUNTIME.robot_id, j)[0], 3) for j in RUNTIME.joint_indices)


def run():
    board = create_initial_board()
    robot = load_robot()
    scene = build_scene(config=C, obstacle_mode="mode_1")

    for mi, mv in enumerate(MOVES):
        print("=" * 64)
        print(f"MOVE {mi+1}: {mv}")
        print(f"  规划前物理关节: {phys()}")
        res = main.run_command(parse_command(mv), board, scene, robot, C)
        ex = res["execution"]

        zz = [-_get_tool0_z_axis(w, C)[2] for w in ex.desired_joint_angles]
        n = len(zz)
        bad_orient = sum(1 for z in zz if z < 0.9)
        print(f"  waypoint 数={n}  朝向不竖直(-zz<0.9)={bad_orient}")
        print(f"  -zz 范围: min={min(zz):.3f} max={max(zz):.3f}")

        je, ee = ex.joint_errors, ex.end_effector_errors
        print(f"  执行 joint_err: max={max(je):.4f} 均={sum(je)/len(je):.4f} rad")
        print(f"  执行 ee_err:    max={max(ee):.4f} 均={sum(ee)/len(ee):.4f} m")
        print(f"  执行后物理关节: {phys()}  耗时={ex.execution_time:.2f}s")

    print("=" * 64)
    print("DONE")


if __name__ == "__main__":
    run()
