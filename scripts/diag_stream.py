"""深入诊断：执行期间运动连续性 profile（DIRECT 模式）。

用途：当感觉走棋卡顿时，用本脚本量化每个 sim step 的关节位移分布，
判断是「连续流动」还是「停-走」。配合 verify_pybullet.py 的检查 3/4 使用。

运行：
    PYTHONPATH=. ./.venv/Scripts/python.exe scripts/diag_stream.py

输出：
    总 sim 步        —— 一条命令的物理步进总数
    空等步占比       —— 单步位移<1e-4 的步占比。逐点沉降模型≈94%(卡顿)；流式应<30%
    每步位移 max/均  —— 关节位移分布。max 过大=瞬移跳变；均值反映平均速度
    段末沉降误差     —— 每段最后实际关节 vs 期望（应<阈值，保证 attach 精度）
"""
from __future__ import annotations

import os
import sys

os.environ.pop("CHESS_ROBOT_PYBULLET_GUI", None)
os.environ["CHESS_ROBOT_BOARD_GUI"] = "0"
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from src.common.config import DEFAULT_CONFIG as C
from src.interaction.board_state import create_initial_board
from src.interaction.cli import parse_command
from src.simulation.load_robot import load_robot
from src.simulation.scene_builder import build_scene
from src.simulation._runtime import RUNTIME, p
import src.control.controller as ctrl
import main

MOVES = ["A4 A5", "C4 C5"]


def run():
    board = create_initial_board()
    robot = load_robot()
    scene = build_scene(config=C, obstacle_mode="mode_1")

    motion = []
    prev = [None]
    _orig = ctrl.p.stepSimulation

    def wstep(*a, **k):
        r = _orig(*a, **k)
        cur = tuple(p.getJointState(RUNTIME.robot_id, j)[0] for j in RUNTIME.joint_indices)
        if prev[0] is not None:
            motion.append(max(abs(cur[k2] - prev[0][k2]) for k2 in range(6)))
        prev[0] = cur
        return r

    ctrl.p.stepSimulation = wstep

    for mv in MOVES:
        motion.clear()
        prev[0] = None
        res = main.run_command(parse_command(mv), board, scene, robot, C)
        ee = res["execution"].end_effector_errors
        tot = len(motion)
        idle = sum(1 for m in motion if m < 1e-4)
        print(f"{mv}: 总sim步={tot}  空等步占比={idle/tot*100:.0f}%  "
              f"每步位移 max={max(motion):.4f} 均={sum(motion)/tot:.5f}  "
              f"段末ee_err≈{ee[-1]:.4f}m")

    ctrl.p.stepSimulation = _orig
    print("DONE")


if __name__ == "__main__":
    run()
