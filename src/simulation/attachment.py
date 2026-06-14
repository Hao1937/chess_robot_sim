from __future__ import annotations

from src.common.config import DEFAULT_CONFIG, Config
from src.common.types import OperationResult
from src.simulation._runtime import RUNTIME, ensure_client, p


def _get_all_piece_body_ids(piece_id: str, client_id: int) -> list[int]:
    """返回棋子的所有 body ID（主 body + 文字标签盘）。

    通过查询 RUNTIME.attachment_constraints 中标签约束的 getConstraintInfo
    来获取标签盘的 child body ID。约束的 key 为 ``{piece_id}_label``。
    """
    body_ids: list[int] = []
    main_id = RUNTIME.piece_body_ids.get(piece_id)
    if main_id is not None:
        body_ids.append(main_id)

    # 查找子部件约束，提取 child body ID
    for key, cid in list(RUNTIME.attachment_constraints.items()):
        if not key.startswith(piece_id + "_"):
            continue
        try:
            info = p.getConstraintInfo(cid, physicsClientId=client_id)
            child_id: int = info[2]  # childBodyUniqueId
            if child_id not in body_ids:
                body_ids.append(child_id)
        except Exception:
            pass

    return body_ids


def attach_piece(
    piece_id: str,
    end_effector_id: int,
    config: Config = DEFAULT_CONFIG,
) -> OperationResult:
    """将棋子（含所有子部件层）吸附到末端执行器吸盘尖端。

    关键设计：
    1. 子部件（trim/top_cap/label）质量设为一个很小的正值（0.001）而非 0——
       Bullet 中 mass=0 是静态刚体，约束求解器无法移动它。
    2. 禁用主 body 碰撞避免棋盘接触力推回棋子。
    3. 棋子应定位在吸盘尖端（suction_cup_link 原点下方 suction_cup_length 处），
       而非连杆原点——JOINT_FIXED 约束的 childFramePosition 将棋子顶部与吸盘尖端对齐。
    4. 瞬移棋子到吸盘尖端对应位置后再创建约束，步进 30 帧让子部件对齐。
    """
    if p is None:
        return OperationResult(True, f"mock attached {piece_id} to end effector {end_effector_id}")

    client_id = ensure_client()
    body_id = RUNTIME.piece_body_ids.get(piece_id)
    if client_id is None or RUNTIME.robot_id is None:
        return OperationResult(True, f"mock attached {piece_id}; robot is not loaded")
    if body_id is None:
        return OperationResult(False, f"unknown piece id: {piece_id}")

    # 移除旧的吸附约束（如果有）
    old_constraint = RUNTIME.attachment_constraints.pop(piece_id, None)
    if old_constraint is not None:
        p.removeConstraint(old_constraint, physicsClientId=client_id)

    # 获取 EE 当前状态
    ee_state = p.getLinkState(RUNTIME.robot_id, end_effector_id, physicsClientId=client_id)
    if ee_state is None:
        return OperationResult(False, f"cannot get end effector link state")

    # 找到所有棋子相关 body（主 body + 子部件）
    all_body_ids = _get_all_piece_body_ids(piece_id, client_id)

    # ── 计算棋子目标位置 ──
    # 棋子应在吸盘尖端下方：棋子顶部接触吸盘尖端，棋子中心在 EE z 下方
    # (suction_cup_length + piece_height/2) 处。
    # 约束 childFramePosition 设置相同偏移量，保持棋子在此位置。
    ee_pos = ee_state[0]
    total_offset = config.suction_cup_length + config.piece_height / 2.0
    piece_target_z = ee_pos[2] - total_offset
    piece_target_pos = (ee_pos[0], ee_pos[1], piece_target_z)

    # ── 关键（Direction A）：在 teleport 之前禁用碰撞并设正质量 ──
    # 必须在任何可能触发碰撞的物理操作之前完成，
    # 避免棋子与棋盘/EE 的接触力将其推离吸盘尖端。
    # mass=0 → 静态刚体 → 约束无法移动！必须保持 mass > 0
    for bid in all_body_ids:
        is_main = (bid == body_id)
        target_mass = 0.001  # 极轻但仍为正，保证动态 + 约束可解
        p.changeDynamics(bid, -1, mass=target_mass, physicsClientId=client_id)
        if is_main:
            # 禁用主 body 碰撞：group=0, mask=0 表示不参与任何碰撞检测
            p.setCollisionFilterGroupMask(bid, -1, 0, 0, physicsClientId=client_id)

    # 瞬移主 body 到吸盘尖端下方
    # 保持 identity 朝向，使 childFramePosition 的 z 偏移沿世界 -z 方向
    p.resetBasePositionAndOrientation(
        body_id,
        piece_target_pos,
        (0.0, 0.0, 0.0, 1.0),
        physicsClientId=client_id,
    )

    # 创建 EE → 主 body 的 JOINT_POINT2POINT 约束（仅约束位置）
    # 相比 JOINT_FIXED，它不强制棋子旋转匹配 EE 朝向。
    # parentFramePosition=(0,0,0)：锚点在 EE 连杆原点
    # childFramePosition=(0, 0, total_offset)：锚点在棋子中心上方 total_offset 处
    # 棋子保持 identity 朝向，+z = 世界 +z，偏移沿世界 z 方向
    # 结果：棋子中心始终在 EE 原点下方 total_offset 处
    try:
        constraint_id = p.createConstraint(
            parentBodyUniqueId=RUNTIME.robot_id,
            parentLinkIndex=end_effector_id,
            childBodyUniqueId=body_id,
            childLinkIndex=-1,
            jointType=p.JOINT_POINT2POINT,
            jointAxis=(0.0, 0.0, 0.0),
            parentFramePosition=(0.0, 0.0, 0.0),
            childFramePosition=(0.0, 0.0, total_offset),
            physicsClientId=client_id,
        )
    except Exception as exc:
        # 约束创建失败时恢复原始动力学
        _restore_piece_dynamics(piece_id, client_id)
        return OperationResult(True, f"mock attached {piece_id}; constraint unavailable: {exc}")
    RUNTIME.attachment_constraints[piece_id] = constraint_id

    # ── 约束沉降阶段 ──
    # 暂时提高求解器迭代次数以更好地满足 JOINT_POINT2POINT + JOINT_FIXED 约束链，
    # 避免沉降后残留 ~2cm 级别的约束违反（R5 诊断证实默认迭代数不足）。
    # 仅在此局部提升，结束后恢复，不影响全局仿真实时性。
    _settle_with_elevated_iterations(client_id, steps=60, elevated_iterations=100)

    # ── 沉降后位置校正（belt-and-suspenders） ──
    # 即使提高了求解器迭代次数，在 EE 倾斜等极端姿态下约束链可能仍有微量残余误差。
    # 沉降完成后将棋子精确瞬移至理论吸盘尖端位置以消除任何残留间隙。
    ee_settled = p.getLinkState(RUNTIME.robot_id, end_effector_id, physicsClientId=client_id)
    if ee_settled is not None:
        ee_final = ee_settled[0]
        corrected_z = ee_final[2] - total_offset
        corrected_pos = (ee_final[0], ee_final[1], corrected_z)
        p.resetBasePositionAndOrientation(
            body_id,
            corrected_pos,
            (0.0, 0.0, 0.0, 1.0),
            physicsClientId=client_id,
        )

    return OperationResult(True, f"attached {piece_id} to end effector {end_effector_id}")


def detach_piece(piece_id: str) -> OperationResult:
    """将棋子从末端执行器释放，恢复原始质量和碰撞属性。"""
    if p is None:
        return OperationResult(True, f"mock detached {piece_id}")

    client_id = ensure_client()
    constraint_id = RUNTIME.attachment_constraints.pop(piece_id, None)
    if client_id is None:
        return OperationResult(True, f"mock detached {piece_id}; no active client")
    if constraint_id is None:
        return OperationResult(True, f"{piece_id} was not attached")

    # 先移除 EE → piece 约束
    p.removeConstraint(constraint_id, physicsClientId=client_id)

    # 恢复棋子动力学属性
    _restore_piece_dynamics(piece_id, client_id)

    return OperationResult(True, f"detached {piece_id}")


def _restore_piece_dynamics(piece_id: str, client_id: int) -> None:
    """将棋子主 body 恢复质量/碰撞。标签盘保持动态质量（0.001）。

    关键：标签盘必须保持 mass > 0（动态），否则释放后主 body 受重力下落时，
    静态标签盘会通过 JOINT_FIXED 约束反向拉扯主 body，导致棋子浮空。
    """
    body_id = RUNTIME.piece_body_ids.get(piece_id)
    if body_id is None:
        return

    all_body_ids = _get_all_piece_body_ids(piece_id, client_id)

    for bid in all_body_ids:
        is_main = (bid == body_id)
        if is_main:
            # 主 body：恢复 mass=0.02 + 碰撞检测
            p.changeDynamics(bid, -1, mass=0.02, physicsClientId=client_id)
            p.setCollisionFilterGroupMask(bid, -1, 1, -1, physicsClientId=client_id)
        else:
            # 标签盘：保持 mass=0.001（动态，确保跟随主 body 下落）
            # 无碰撞形状，无需调整碰撞过滤
            p.changeDynamics(bid, -1, mass=0.001, physicsClientId=client_id)


def _settle_with_elevated_iterations(
    client_id: int,
    steps: int = 60,
    elevated_iterations: int = 100,
) -> None:
    """临时提高求解器迭代次数运行指定步数，完成后恢复默认值。

    仅在约束创建后的沉降阶段使用，避免全局提高迭代次数导致
    仿真实时性下降（R5 教训：全局 500 次迭代造成 GUI 明显卡顿）。
    """
    try:
        # 保存当前迭代次数
        original_iterations = p.getPhysicsEngineParameter(
            "numSolverIterations", physicsClientId=client_id
        )
    except Exception:
        original_iterations = None

    try:
        p.setPhysicsEngineParameter(
            numSolverIterations=elevated_iterations, physicsClientId=client_id
        )
        for _ in range(steps):
            p.stepSimulation(client_id)
    finally:
        # 恢复原始迭代次数
        if original_iterations is not None:
            try:
                p.setPhysicsEngineParameter(
                    numSolverIterations=original_iterations,
                    physicsClientId=client_id,
                )
            except Exception:
                pass
