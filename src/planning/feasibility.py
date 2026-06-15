from __future__ import annotations

import math

from src.common.config import Config
from src.common.types import JointTrajectory, PrimitivePlanningContext
from src.control.fk_solver import solve_fk, _get_tool0_z_axis
from src.planning.collision_checker import check_segment_collision


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

    # ── a) IK 可达性检查 ──
    # 仅检查每个 primitive 的终点 waypoint：
    # 中间插值点从 home_pose 过渡时朝向可能暂时不佳，
    # 只要到达目标点时 -zz 和位置误差符合要求即可。
    for i, waypoint in enumerate(trajectory.joint_waypoints):
        if primitive_ranges is not None:
            prim_idx = _find_primitive_idx(i, primitive_ranges, num_contexts)
        else:
            prim_idx = min(i, num_contexts - 1) if num_contexts > 0 else 0

        if prim_idx >= num_contexts:
            continue

        # 判断是否为本 primitive 的终点 waypoint
        if primitive_ranges is not None and prim_idx < len(primitive_ranges):
            _, end = primitive_ranges[prim_idx]
            is_last = (i == end - 1)
        else:
            is_last = (i == len(trajectory.joint_waypoints) - 1)

        if not is_last:
            continue

        # 计算 FK 位置和 tool0 z 轴
        fk_xyz = solve_fk(waypoint, config)
        z_axis = _get_tool0_z_axis(waypoint, config)
        zz = z_axis[2]

        # 检查 tool 朝向：仅对关键操作 primitive（descend/lift/grasp/detach）
        # approach/transfer 在高空行进，朝向要求可放宽
        prim_type = planning_contexts[prim_idx].primitive.primitive_type
        if prim_type in ("descend", "lift", "grasp", "detach"):
            if -zz < config.feasibility_zz_min:
                return (False, f"不可达：waypoint {i} pos_err=N/A -zz={-zz:.3f}")

        # 检查位置误差
        target_xyz = planning_contexts[prim_idx].primitive.target_xyz
        pos_err = math.sqrt(
            (fk_xyz[0] - target_xyz[0]) ** 2
            + (fk_xyz[1] - target_xyz[1]) ** 2
            + (fk_xyz[2] - target_xyz[2]) ** 2
        )
        if pos_err > config.feasibility_pos_tol:
            return (
                False,
                f"不可达：waypoint {i} pos_err={pos_err:.3f} -zz={-zz:.3f}",
            )

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


def _find_primitive_idx(
    waypoint_idx: int,
    primitive_ranges: list[tuple[int, int]],
    num_contexts: int,
) -> int:
    """查找某 waypoint 索引属于哪个 primitive 范围。"""
    for prim_idx, (start, end) in enumerate(primitive_ranges):
        if start <= waypoint_idx < end:
            return prim_idx
    return num_contexts - 1  # fallback
