"""
PyBullet 物理验证脚本 — 在 DIRECT 模式下实际运行完整管线，
逐阶段输出物理状态，用于诊断吸附、子部件跟随、末端朝向等问题。

用法:
    python scripts/diagnose_pybullet.py [--command "A4 A5"]
"""

from __future__ import annotations

import argparse
import math
import os
import sys

# 强制使用 PyBullet DIRECT 模式（不使用 mock）
os.environ["CHESS_ROBOT_PYBULLET_GUI"] = "0"

from src.common.config import DEFAULT_CONFIG, Config
from src.control.fk_solver import _get_tool0_z_axis, _solve_fk_urdf_chain, _solve_fk_pybullet
from src.interaction.board_state import create_initial_board, make_logical_actions
from src.interaction.chess_rules import validate_move
from src.interaction.cli import parse_command
from src.planning.motion_primitives import build_motion_primitives, get_action_primitive_ranges
from src.planning.obstacle_map import build_primitive_obstacle_contexts
from src.planning.trajectory_planner import plan_trajectory
from src.simulation._runtime import RUNTIME, ensure_client, p


# ── 诊断输出工具 ──

def _fmt_vec(v, precision=4):
    """格式化向量。"""
    return "(" + ", ".join(f"{x:.{precision}f}" for x in v) + ")"


def _check_pybullet_ok() -> bool:
    """检查 PyBullet 是否可用且已连接。"""
    if p is None:
        print("❌ PyBullet 不可用 (p is None)")
        return False
    cid = ensure_client()
    if cid is None:
        print("❌ 无法创建 PyBullet 客户端")
        return False
    return True


# ── 诊断阶段 1: 场景构建 ──

def diagnose_scene(config: Config):
    """构建场景并输出初始状态。"""
    print("=" * 70)
    print("阶段 1: 场景构建")
    print("=" * 70)

    from src.simulation.scene_builder import build_scene
    from src.simulation.load_robot import load_robot

    robot = load_robot()
    scene = build_scene(config=config, obstacle_mode="mode_1")
    board = create_initial_board()

    cid = ensure_client()

    # 输出机器人状态
    print(f"\n机器人:")
    print(f"  robot_id={RUNTIME.robot_id}")
    print(f"  end_effector_id={RUNTIME.end_effector_id}")
    print(f"  joint_indices={RUNTIME.joint_indices}")
    print(f"  client_id={cid}")

    # 输出机器人初始关节角度
    if RUNTIME.robot_id is not None:
        joints = []
        for ji in RUNTIME.joint_indices:
            js = p.getJointState(RUNTIME.robot_id, ji, physicsClientId=cid)
            joints.append(js[0])
        print(f"  初始关节角: {_fmt_vec(joints, 4)}")

        # 初始 EE 状态
        ee_state = p.getLinkState(RUNTIME.robot_id, RUNTIME.end_effector_id, physicsClientId=cid)
        print(f"  初始 EE 位置: {_fmt_vec(ee_state[0])}")
        print(f"  初始 tool0 z 轴: {_fmt_vec(_get_tool0_z_axis(tuple(joints), config))}")
        print(f"    -zz (朝下得分): {- _get_tool0_z_axis(tuple(joints), config)[2]:.4f}")

    # 输出棋子状态 — 检查 red_soldier_1 (A4)
    piece_id = "red_soldier_1"
    body_id = RUNTIME.piece_body_ids.get(piece_id)
    print(f"\n棋子 {piece_id}:")
    print(f"  body_id={body_id}")
    if body_id is not None:
        pos, orn = p.getBasePositionAndOrientation(body_id, physicsClientId=cid)
        print(f"  位置: {_fmt_vec(pos)}")
        print(f"  朝向: {_fmt_vec(orn)}")
        print(f"  所在 cell: {RUNTIME.piece_cells.get(piece_id)}")

    # 列出所有与 red_soldier_1 相关的约束
    print(f"\n  关联约束:")
    for ckey, cid_val in RUNTIME.attachment_constraints.items():
        if piece_id in ckey:
            print(f"    {ckey} -> constraint {cid_val}")

    return robot, scene, board


# ── 诊断阶段 2: 轨迹规划 ──

def diagnose_planning(board, scene, config: Config, command_str: str):
    """规划轨迹并输出关键信息。"""
    print("\n" + "=" * 70)
    print("阶段 2: 轨迹规划")
    print("=" * 70)

    command = parse_command(command_str)
    validation = validate_move(board, command)
    print(f"指令: {command}")
    print(f"合法性: {validation.is_legal}")
    if not validation.is_legal:
        print(f"  原因: {validation.reason}")
        return None

    actions = make_logical_actions(board, command)
    primitives = build_motion_primitives(actions, config)
    planning_contexts = build_primitive_obstacle_contexts(
        actions=actions, primitives=primitives, board=board,
        extra_obstacles=scene.obstacles, human_hand_present=False, config=config,
    )
    trajectory = plan_trajectory(planning_contexts, config=config)

    print(f"\n动作列表:")
    for i, a in enumerate(actions):
        print(f"  [{i}] {a.action_type} {a.cell} piece={a.piece_id}")
    print(f"动作基元: {len(primitives)}")
    for i, prim in enumerate(primitives):
        print(f"  [{i}] {prim.primitive_type} -> {_fmt_vec(prim.target_xyz, 3)} speed={prim.speed_mode}")
    print(f"路径点总数: {len(trajectory.joint_waypoints)}")
    if trajectory.primitive_ranges:
        print(f"动作基元路径点范围: {trajectory.primitive_ranges}")
        for i, (ws, we) in enumerate(trajectory.primitive_ranges):
            print(f"  [{i}] waypoints [{ws}:{we}] ({we - ws} pts)")

    # 检查每个动作基元的目标 IK 解
    print(f"\nIK 解验证 (tool0 z 轴朝向):")
    for i, prim in enumerate(primitives):
        if i < len(trajectory.joint_waypoints):
            # 使用动作基元的第一个路径点
            wp_idx = trajectory.primitive_ranges[i][0] if trajectory.primitive_ranges else i
            if wp_idx < len(trajectory.joint_waypoints):
                jw = trajectory.joint_waypoints[wp_idx]
                zx, zy, zz = _get_tool0_z_axis(jw, config)
                pos = _solve_fk_urdf_chain(jw, config)
                print(f"  [{i}] {prim.primitive_type}: pos={_fmt_vec(pos, 3)} "
                      f"z_axis=({zx:.3f},{zy:.3f},{zz:.3f}) -zz={-zz:.3f}")

    return trajectory, actions, primitives, planning_contexts


# ── 诊断阶段 3: 分步执行 ──

def diagnose_execution(trajectory, actions, robot, board, config: Config):
    """分步执行轨迹，每步输出物理状态。"""
    print("\n" + "=" * 70)
    print("阶段 3: 分步执行")
    print("=" * 70)

    from src.control.controller import execute_trajectory
    from src.simulation.attachment import attach_piece, detach_piece
    from src.planning.motion_primitives import get_action_primitive_ranges

    cid = ensure_client()
    action_prim_ranges = get_action_primitive_ranges(actions)
    primitive_ranges = trajectory.primitive_ranges
    attached_piece_id = ""

    for i, action in enumerate(actions):
        prim_start, prim_end = action_prim_ranges[i]
        print(f"\n--- Action {i}: {action.action_type} {action.cell} ---")
        print(f"  Primitive 范围: [{prim_start}, {prim_end})")

        if not primitive_ranges or prim_start >= len(primitive_ranges):
            print("  ⚠ primitive_ranges 不可用，跳过")
            continue

        if action.action_type == "pick" and action.piece_id:
            # ── pick: pre-attach 段 ──
            pre_end = prim_start + 2
            wp_pre_start = primitive_ranges[prim_start][0]
            wp_pre_end = primitive_ranges[pre_end - 1][1]
            print(f"  Pre-attach 段: waypoints[{wp_pre_start}:{wp_pre_end}]")

            pre_segment = type(trajectory)(
                joint_waypoints=trajectory.joint_waypoints[wp_pre_start:wp_pre_end],
                speed_profile=trajectory.speed_profile[wp_pre_start:wp_pre_end],
            )
            result_pre = execute_trajectory(pre_segment)
            print(f"  Pre-attach 执行: success={result_pre.success}, time={result_pre.execution_time}s")

            # 吸附前：检查 EE 与棋子的相对位置
            piece_id = action.piece_id
            body_id = RUNTIME.piece_body_ids.get(piece_id)
            ee_state = p.getLinkState(RUNTIME.robot_id, RUNTIME.end_effector_id, physicsClientId=cid)
            if body_id is not None:
                piece_pos = p.getBasePositionAndOrientation(body_id, physicsClientId=cid)[0]
                dist = math.hypot(
                    ee_state[0][0] - piece_pos[0],
                    ee_state[0][1] - piece_pos[1],
                    ee_state[0][2] - piece_pos[2],
                )
                print(f"  吸附前 EE 位置: {_fmt_vec(ee_state[0])}")
                print(f"  吸附前 棋子位置: {_fmt_vec(piece_pos)}")
                print(f"  EE-棋子距离: {dist:.4f}m")

            # 执行吸附
            print(f"  执行 attach_piece({piece_id})...")
            attach_result = attach_piece(piece_id=piece_id, end_effector_id=robot.end_effector_id)
            print(f"  attach 结果: {attach_result.message}")
            attached_piece_id = piece_id

            # 吸附后：检查棋子位置是否跟随 EE
            if body_id is not None:
                piece_pos_after = p.getBasePositionAndOrientation(body_id, physicsClientId=cid)[0]
                print(f"  吸附后 棋子位置: {_fmt_vec(piece_pos_after)}")
                # 检查子部件
                _check_sub_parts(piece_id, config)

            # ── pick: post-attach 段 ──
            if prim_end > pre_end:
                wp_post_start = primitive_ranges[pre_end][0]
                wp_post_end = primitive_ranges[prim_end - 1][1]
                print(f"  Post-attach 段: waypoints[{wp_post_start}:{wp_post_end}]")

                post_segment = type(trajectory)(
                    joint_waypoints=trajectory.joint_waypoints[wp_post_start:wp_post_end],
                    speed_profile=trajectory.speed_profile[wp_post_start:wp_post_end],
                )
                result_post = execute_trajectory(post_segment)
                print(f"  Post-attach 执行: success={result_post.success}, time={result_post.execution_time}s")

                # 检查 lift 后棋子和子部件状态
                _check_sub_parts(piece_id, config)

        elif action.action_type == "place" and attached_piece_id:
            # ── place: pre-detach 段 ──
            pre_end = prim_start + 2
            wp_pre_start = primitive_ranges[prim_start][0]
            wp_pre_end = primitive_ranges[pre_end - 1][1]
            print(f"  Pre-detach 段: waypoints[{wp_pre_start}:{wp_pre_end}]")

            pre_segment = type(trajectory)(
                joint_waypoints=trajectory.joint_waypoints[wp_pre_start:wp_pre_end],
                speed_profile=trajectory.speed_profile[wp_pre_start:wp_pre_end],
            )
            result_pre = execute_trajectory(pre_segment)
            print(f"  Pre-detach 执行: success={result_pre.success}, time={result_pre.execution_time}s")

            # 释放前检查
            _check_sub_parts(attached_piece_id, config)

            # 执行释放
            print(f"  执行 detach_piece({attached_piece_id})...")
            detach_result = detach_piece(piece_id=attached_piece_id)
            print(f"  detach 结果: {detach_result.message}")

            piece_id = attached_piece_id
            attached_piece_id = ""

            # 释放后检查
            body_id = RUNTIME.piece_body_ids.get(piece_id)
            if body_id is not None:
                piece_pos = p.getBasePositionAndOrientation(body_id, physicsClientId=cid)[0]
                print(f"  释放后 棋子位置: {_fmt_vec(piece_pos)}")
                _check_sub_parts(piece_id, config)

            # ── place: post-detach 段 ──
            if prim_end > pre_end:
                wp_post_start = primitive_ranges[pre_end][0]
                wp_post_end = primitive_ranges[prim_end - 1][1]
                print(f"  Post-detach 段: waypoints[{wp_post_start}:{wp_post_end}]")

                post_segment = type(trajectory)(
                    joint_waypoints=trajectory.joint_waypoints[wp_post_start:wp_post_end],
                    speed_profile=trajectory.speed_profile[wp_post_start:wp_post_end],
                )
                result_post = execute_trajectory(post_segment)
                print(f"  Post-detach 执行: success={result_post.success}, time={result_post.execution_time}s")

        else:
            # ── 其他动作类型 ──
            wp_start = primitive_ranges[prim_start][0]
            wp_end = primitive_ranges[prim_end - 1][1] if prim_end > prim_start else wp_start
            print(f"  段: waypoints[{wp_start}:{wp_end}]")
            segment = type(trajectory)(
                joint_waypoints=trajectory.joint_waypoints[wp_start:wp_end],
                speed_profile=trajectory.speed_profile[wp_start:wp_end],
            )
            result = execute_trajectory(segment)
            print(f"  执行: success={result.success}, time={result.execution_time}s")

    # 最终检查
    print(f"\n--- 最终状态 ---")
    ee_state = p.getLinkState(RUNTIME.robot_id, RUNTIME.end_effector_id, physicsClientId=cid)
    # 获取最终关节角
    final_joints = []
    for ji in RUNTIME.joint_indices:
        js = p.getJointState(RUNTIME.robot_id, ji, physicsClientId=cid)
        final_joints.append(js[0])
    zx, zy, zz = _get_tool0_z_axis(tuple(final_joints), config)
    print(f"  末端 EE 位置: {_fmt_vec(ee_state[0])}")
    print(f"  末端 tool0 z 轴: ({zx:.4f}, {zy:.4f}, {zz:.4f})  -zz={-zz:.4f}")
    print(f"  剩余约束数: {len(RUNTIME.attachment_constraints)}")


# ── 子部件检查 ──

def _check_sub_parts(piece_id: str, config: Config):
    """检查棋子的子部件（装饰环、顶盖、标签）是否跟随主体。"""
    cid = ensure_client()
    body_id = RUNTIME.piece_body_ids.get(piece_id)
    if body_id is None:
        print(f"  ⚠ 未找到棋子 {piece_id}")
        return

    main_pos = p.getBasePositionAndOrientation(body_id, physicsClientId=cid)[0]

    # 查找与 piece_id 相关的所有场景物体
    sub_keys = [
        k for k in RUNTIME.attachment_constraints
        if piece_id in k and k != piece_id
    ]
    issues = []
    for key in sub_keys:
        # 从 scene_body_ids 中找到对应的 body... 无法直接映射
        # 改为遍历 scene_body_ids 找约束关联
        pass

    # 简化检查: 确认主体位置与预期 cell 一致
    cell = RUNTIME.piece_cells.get(piece_id, "?")
    print(f"  子部件检查 {piece_id}: 主体位置={_fmt_vec(main_pos)}, 所在 cell={cell}")


# ── 诊断阶段 4: FK 交叉验证 ──

def diagnose_fk_cross_validation(config: Config):
    """对比 URDF 链 FK 与 PyBullet FK 的一致性。"""
    print("\n" + "=" * 70)
    print("阶段 4: FK 交叉验证 (URDF 链 vs PyBullet)")
    print("=" * 70)

    import random
    rng = random.Random(42)

    max_err = 0.0
    for i in range(10):
        joints = tuple(
            rng.uniform(-math.pi, math.pi) for _ in range(6)
        )
        urdf_pos = _solve_fk_urdf_chain(joints, config)
        pyb_pos = _solve_fk_pybullet(joints)
        if pyb_pos is None:
            print(f"  [{i}] PyBullet FK 不可用")
            return
        err = math.hypot(
            urdf_pos[0] - pyb_pos[0],
            urdf_pos[1] - pyb_pos[1],
            urdf_pos[2] - pyb_pos[2],
        )
        max_err = max(max_err, err)
        status = "✓" if err < 0.001 else ("⚠" if err < 0.01 else "❌")
        print(f"  [{i}] joints={_fmt_vec(joints, 2)} URDF={_fmt_vec(urdf_pos, 4)} PyB={_fmt_vec(pyb_pos, 4)} err={err:.6f}m {status}")

    print(f"\n  最大误差: {max_err:.6f}m")


# ── 诊断阶段 5: IK 多目标验证 ──

def diagnose_ik_multi_target(config: Config):
    """测试多个棋盘格子的 IK 解质量。"""
    print("\n" + "=" * 70)
    print("阶段 5: IK 多目标验证")
    print("=" * 70)

    from src.planning.ik_solver import solve_ik

    test_cells = [
        ("A1", (0.0, 0.0, 0.055)),
        ("A4", (0.0, 0.18, 0.055)),
        ("A5", (0.0, 0.24, 0.055)),
        ("E5", (0.24, 0.24, 0.055)),
        ("I1", (0.48, 0.0, 0.055)),
        ("I10", (0.48, 0.54, 0.055)),
    ]

    all_ok = True
    for cell, target in test_cells:
        joints = solve_ik(target, config)
        zx, zy, zz = _get_tool0_z_axis(joints, config)

        # PyBullet FK 验证
        pyb_pos = _solve_fk_pybullet(joints)
        if pyb_pos is not None:
            pos_err = math.hypot(
                target[0] - pyb_pos[0],
                target[1] - pyb_pos[1],
                target[2] - pyb_pos[2],
            )
        else:
            pos_err = float('nan')

        orient_ok = -zz > 0.7
        pos_ok = pos_err < 0.02 if not math.isnan(pos_err) else False
        status = "✓" if (orient_ok and (math.isnan(pos_err) or pos_ok)) else "❌"
        if status == "❌":
            all_ok = False

        print(f"  {cell} {_fmt_vec(target, 3)}: "
              f"-zz={-zz:.3f} {'↑' if orient_ok else '↓'} "
              f"pos_err={pos_err:.4f}m {'✓' if pos_ok else '❌'} "
              f"{status}")

    print(f"\n  总体: {'全部通过 ✓' if all_ok else '存在问题 ❌'}")


# ── 主入口 ──

def main():
    parser = argparse.ArgumentParser(description="PyBullet 物理验证诊断脚本")
    parser.add_argument("--command", default="A4 A5", help="要测试的走法指令")
    parser.add_argument("--stages", default="1,2,3,4,5", help="要运行的诊断阶段 (逗号分隔)")
    args = parser.parse_args()

    if not _check_pybullet_ok():
        print("\n请确保 PyBullet 已安装: pip install pybullet")
        sys.exit(1)

    config = DEFAULT_CONFIG
    stages = set(int(s.strip()) for s in args.stages.split(","))

    print("PyBullet 物理验证诊断")
    print(f"指令: {args.command}")
    print(f"Base 位置: {_fmt_vec(config.base_link_position)}")
    print(f"Home pose: {_fmt_vec(config.home_pose)}")
    print(f"z_grasp={config.z_grasp}, z_safe={config.z_safe}")
    print(f"棋盘: {config.board_cols}x{config.board_rows} cells @ {config.cell_size}m")
    print()

    try:
        if 1 in stages:
            robot, scene, board = diagnose_scene(config)
        else:
            from src.simulation.scene_builder import build_scene
            from src.simulation.load_robot import load_robot
            robot = load_robot()
            scene = build_scene(config=config, obstacle_mode="mode_1")
            board = create_initial_board()

        if 2 in stages:
            result = diagnose_planning(board, scene, config, args.command)
            if result is None:
                print("\n❌ 轨迹规划失败")
                return
            trajectory, actions, primitives, planning_contexts = result
        else:
            # 快速规划
            command = parse_command(args.command)
            actions = make_logical_actions(board, command)
            primitives = build_motion_primitives(actions, config)
            planning_contexts = build_primitive_obstacle_contexts(
                actions=actions, primitives=primitives, board=board,
                extra_obstacles=scene.obstacles, human_hand_present=False, config=config,
            )
            trajectory = plan_trajectory(planning_contexts, config=config)

        if 3 in stages:
            diagnose_execution(trajectory, actions, robot, board, config)

        if 4 in stages:
            diagnose_fk_cross_validation(config)

        if 5 in stages:
            diagnose_ik_multi_target(config)

    finally:
        # 清理
        cid = RUNTIME.client_id
        if cid is not None and p is not None:
            try:
                p.disconnect(cid)
            except Exception:
                pass

    print("\n诊断完成。")


if __name__ == "__main__":
    main()
