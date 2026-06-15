from __future__ import annotations

import math

from src.common.config import Config
from src.common.types import JointTrajectory, PrimitivePlanningContext
from src.control.fk_solver import solve_fk, _get_tool0_z_axis
from src.planning.collision_checker import check_segment_collision
from src.planning.ik_solver import solve_ik, is_reachable


def validate_trajectory_feasibility(
    planning_contexts: list[PrimitivePlanningContext],
    trajectory: JointTrajectory,
    config: Config,
) -> tuple[bool, str]:
    """验证轨迹可行性：IK 可达性、路径碰撞、落点合法性。

    Args:
        planning_contexts: 每个 motion primitive 的规划上下文（含障碍物信息）
        trajectory: 已规划的关节轨迹
        config: 配置对象

    Returns:
        (True, "ok") 全部通过, (False, 原因) 首次失败
    """
    primitive_ranges = trajectory.primitive_ranges
    num_contexts = len(planning_contexts)

    # ── a) 目标可达性检查 ──
    # 关键修正（2026-06-15 第二轮验收）：检查每个 primitive 的**目标点**本身是否
    # 几何可达，而非检查链式/平滑轨迹上某个 waypoint 的 pos_err。
    #
    # 旧实现的缺陷：用链式 IK 轨迹 waypoint 的 pos_err 当作"可达性"判据。
    # 链式种子在捕获等长序列中会经过别扭中间姿态，个别 waypoint 误差偏大；
    # 加上 waypoint↔primitive 索引映射脆弱，导致**几何可达的目标被误判为不可达**
    # （用户目击的 I 列误拒）。实测：这些目标用 home 种子单独求解 pos_err≈0.0002。
    #
    # 正确做法：对每个目标用 home 种子**新鲜求解**，测的是"目标本身能否到达"
    # （这才是"路线不可达"该回答的问题），不受链式/平滑/索引伪差影响。
    # 真正不可达的目标（超出工作空间、被柱占据）用新鲜求解同样会失败 → 仍被拦截。
    for ctx in planning_contexts:
        prim = ctx.primitive
        prim_type = prim.primitive_type
        if prim_type not in (
            "approach", "transfer", "descend", "lift", "grasp", "detach", "retreat",
        ):
            continue
        target_xyz = prim.target_xyz

        # 几何工作空间硬限（内圈 0.25m + 外圈 0.9m）
        if not is_reachable(target_xyz, config):
            return (False, f"不可达：{prim_type} 目标超出机械臂工作空间")

        # 新鲜 IK 求解，验证目标真实可达
        joints = solve_ik(target_xyz, config, seed=config.home_pose[:6])
        fk_xyz = solve_fk(joints, config)
        pos_err = math.sqrt(
            (fk_xyz[0] - target_xyz[0]) ** 2
            + (fk_xyz[1] - target_xyz[1]) ** 2
            + (fk_xyz[2] - target_xyz[2]) ** 2
        )
        if pos_err > config.feasibility_pos_tol:
            return (False, f"不可达：{prim_type} 目标 IK 误差 {pos_err:.3f}m")

        # 关键操作（descend/lift/grasp/detach）需吸盘朝向不灾难性退化
        if prim_type in ("descend", "lift", "grasp", "detach"):
            zz = -_get_tool0_z_axis(joints, config)[2]
            if zz < config.feasibility_zz_min:
                return (False, f"不可达：{prim_type} 目标吸盘朝向 -zz={zz:.3f}")

    # ── b) 水平段路径碰撞检查 ──
    # 对 approach/transfer 素，检查相邻 waypoint 之间的 cartesian 路径
    # 是否与障碍物碰撞（使用该 primitive 对应时刻的障碍物列表）。
    if primitive_ranges is not None:
        for prim_idx, (seg_start, seg_end) in enumerate(primitive_ranges):
            if prim_idx >= num_contexts:
                break
            prim_type = planning_contexts[prim_idx].primitive.primitive_type
            if prim_type not in ("approach", "transfer"):
                continue

            obstacles = planning_contexts[prim_idx].obstacles

            # 逐段检查相邻 waypoint 之间的直线路径
            for wp_idx in range(seg_start, seg_end - 1):
                fk_a = solve_fk(trajectory.joint_waypoints[wp_idx], config)
                fk_b = solve_fk(trajectory.joint_waypoints[wp_idx + 1], config)

                # 过滤障碍物：只考虑高度达到路径最低点的障碍物
                # （棋子高度 0.018，transfer 在 z_safe=0.18，不应被棋子阻挡）
                path_min_z = min(fk_a[2], fk_b[2])
                relevant = [o for o in obstacles
                            if o.height >= path_min_z - config.safety_margin]

                if not relevant:
                    continue

                result = check_segment_collision(
                    fk_a,
                    fk_b,
                    relevant,
                    step_size=config.path_collision_check_step,
                    safety_margin=config.safety_margin,
                )
                if not result.collision_free:
                    return (False, "不可达：路径穿障")

    # ── c) 落点合法性检查 ──
    # 对 descend primitive，检查目标点 (x, y, z_grasp) 是否在
    # 任何障碍物的膨胀半径内。
    # 注意：不检查 piece_ 障碍物。descend 可能发生在 pick 操作中，
    # 此时目标点就是棋子上方，棋子自身作为障碍物预期会阻挡落点，
    # 这是正常的下降轨迹，不应阻断。
    if primitive_ranges is not None:
        for prim_idx, (seg_start, seg_end) in enumerate(primitive_ranges):
            if prim_idx >= num_contexts:
                break
            prim_type = planning_contexts[prim_idx].primitive.primitive_type
            if prim_type != "descend":
                continue

            target_xyz = planning_contexts[prim_idx].primitive.target_xyz
            obstacles = planning_contexts[prim_idx].obstacles
            tx, ty = target_xyz[0], target_xyz[1]

            for obstacle in obstacles:
                # 排除棋子障碍物：pick 操作的 descend 目标就是棋子上方
                if obstacle.obstacle_id.startswith("piece_"):
                    continue
                ox, oy = obstacle.center_xyz[0], obstacle.center_xyz[1]
                dist = math.sqrt((tx - ox) ** 2 + (ty - oy) ** 2)
                inflated_r = obstacle.radius + config.safety_margin
                if dist < inflated_r:
                    return (False, "不可达：落点被障碍物阻挡")

    return (True, "ok")
