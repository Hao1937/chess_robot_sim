"""调研：(a)避障绕行是否IK可行 (b)base位置可达性分析。仅诊断。"""
from __future__ import annotations
import os, math
os.environ.pop("CHESS_ROBOT_PYBULLET_GUI", None)
os.environ["CHESS_ROBOT_BOARD_GUI"] = "0"
import sys
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from src.common.config import DEFAULT_CONFIG as C
from src.common.types import MotionPrimitive, Obstacle
from src.planning.trajectory_planner import plan_trajectory
from src.planning.collision_checker import check_segment_collision_multi_z
from src.planning.ik_solver import solve_ik
from src.control.fk_solver import _solve_fk_urdf_chain, _get_tool0_z_axis
from src.simulation.load_robot import load_robot
from src.simulation.scene_builder import build_scene
from src.simulation._runtime import RUNTIME, p
import main
from src.interaction.board_state import create_initial_board
from src.interaction.cli import parse_command

board = create_initial_board(); robot = load_robot(); scene = build_scene(config=C, obstacle_mode="mode_1")
obs = scene.obstacles

print("##### (a) E4->E5 单独跑（fresh，排除接在失败后的干扰）#####")
r = main.run_command(parse_command("E4 E5"), board, scene, robot, C)
ee = r["execution"].end_effector_errors
print(f"  E4->E5 fresh: ee_err max={max(ee):.4f} mean={sum(ee)/len(ee):.4f}")

print("##### (b) 直接规划穿越障碍的 transfer：(0.24,0.12)->(0.24,0.48) z=0.18 #####")
# 障碍在 (0.24,0.30)。直线必穿障。
start=(0.24,0.12,0.18); end=(0.24,0.48,0.18)
clear = check_segment_collision_multi_z((start[0],start[1]),(end[0],end[1]),0.18,obs,
                                        step_size=C.path_collision_check_step, safety_margin=C.safety_margin)
print(f"  直线无碰撞? {clear}（应为 False，需绕行）")
prims=[MotionPrimitive("approach","E3",start,"fast",None),
       MotionPrimitive("transfer","E9",end,"fast",None)]
traj=plan_trajectory(prims, obstacles=obs, config=C)
wps=traj.joint_waypoints
# 绕行 EE 轨迹是否真的避开障碍 + IK 是否可行
max_xy_into=0.0; bad_ik=0
for jw in wps:
    eepos=_solve_fk_urdf_chain(jw,C); zz=-_get_tool0_z_axis(jw,C)[2]
    pe=0.0  # 无逐点 cart 对照
    for o in obs:
        d=math.hypot(eepos[0]-o.center_xyz[0], eepos[1]-o.center_xyz[1])
        pen=o.radius-d
        if eepos[2]<o.height and pen>max_xy_into: max_xy_into=pen
    if zz<0.85: bad_ik+=1
print(f"  绕行轨迹 {len(wps)} wp | EE最大侵入障碍={max_xy_into:.4f}m(负或0=避开) | 朝向差wp={bad_ik}")

print("##### (c) base 可达性分析：当前 base vs 候选 base #####")
def analyze_base(bx,by,bz):
    from dataclasses import replace
    cfg=replace(C, base_link_position=(bx,by,bz))
    # 需要机器人实际在该 base 才能用 pybullet FK；这里用 URDF 链 FK（解析，含 base 偏移）
    worst_zz=1.0; worst_pe=0.0; cnt_bad=0; total=0
    cols='ABCDEFGHI'
    for ci in range(9):
        for ri in range(1,11):
            tgt=(ci*0.06, (ri-1)*0.06, C.z_grasp)
            dist=math.hypot(tgt[0]-bx, tgt[1]-by)
            total+=1
            j=solve_ik(tgt,cfg,seed=cfg.home_pose[:6])
            ee=_solve_fk_urdf_chain(j,cfg); zz=-_get_tool0_z_axis(j,cfg)[2]
            pe=math.dist(ee,tgt)
            worst_zz=min(worst_zz,zz); worst_pe=max(worst_pe,pe)
            if zz<0.85 or pe>0.02: cnt_bad+=1
    print(f"  base=({bx:.2f},{by:.2f}): 最差-zz={worst_zz:.3f} 最大pos_err={worst_pe:.3f} 不良格数={cnt_bad}/{total}")

# 注意：analyze_base 用解析 FK，需在 load_robot 之前不影响；这里直接算
# 当前 base (0.24,-0.25,0.12)
analyze_base(0.24,-0.25,0.12)
# 候选：base 移到棋盘中心线、离板更近（y 更靠正中，减少最远行距离）
analyze_base(0.24,-0.10,0.12)
analyze_base(0.24,0.00,0.12)
analyze_base(0.24,0.27,0.12)   # 正对棋盘中心(y=0.27)
print("DONE")
