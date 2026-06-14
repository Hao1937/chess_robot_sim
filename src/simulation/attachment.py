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

    采用**手动位置更新**方案（非 PyBullet 约束方案）：
    1. 子部件质量设为很小的正值（0.001）保持动态。
    2. 禁用主 body 碰撞避免棋盘接触力推回棋子。
    3. Teleport 棋子到吸盘尖端下方，注册到 manually_attached_pieces。
    4. 棋子跟随 EE 由 sync_manual_attachments() 在每个仿真步后同步，
       精度 0mm，不受约束求解器迭代次数影响。
    """
    if p is None:
        return OperationResult(True, f"mock attached {piece_id} to end effector {end_effector_id}")

    client_id = ensure_client()
    body_id = RUNTIME.piece_body_ids.get(piece_id)
    if client_id is None or RUNTIME.robot_id is None:
        return OperationResult(True, f"mock attached {piece_id}; robot is not loaded")
    if body_id is None:
        return OperationResult(False, f"unknown piece id: {piece_id}")

    # 移除旧的吸附方式（约束或手动）
    old_constraint = RUNTIME.attachment_constraints.pop(piece_id, None)
    if old_constraint is not None:
        p.removeConstraint(old_constraint, physicsClientId=client_id)
    RUNTIME.manually_attached_pieces.pop(piece_id, None)

    # 获取 EE 当前状态
    ee_state = p.getLinkState(RUNTIME.robot_id, end_effector_id, physicsClientId=client_id)
    if ee_state is None:
        return OperationResult(False, f"cannot get end effector link state")

    # 找到所有棋子相关 body（主 body + 子部件）
    all_body_ids = _get_all_piece_body_ids(piece_id, client_id)

    # ── 计算棋子目标位置 —— 吸盘尖端下方 piece_height/2 ──
    ee_pos = ee_state[0]
    ee_orn = ee_state[1]
    pad_tip_world = _transform_point(
        (0.0, 0.0, config.suction_cup_length), ee_pos, ee_orn
    )
    piece_target_pos = (
        pad_tip_world[0],
        pad_tip_world[1],
        pad_tip_world[2] - config.piece_height / 2.0,
    )

    # ── 禁用碰撞并设动态质量 ──
    for bid in all_body_ids:
        is_main = (bid == body_id)
        target_mass = 0.001  # 极轻但仍为正，保证动态
        p.changeDynamics(bid, -1, mass=target_mass, physicsClientId=client_id)
        if is_main:
            # 禁用主 body 碰撞：group=0, mask=0 表示不参与任何碰撞检测
            p.setCollisionFilterGroupMask(bid, -1, 0, 0, physicsClientId=client_id)

    # 瞬移主 body 到吸盘尖端下方（identity 朝向）
    p.resetBasePositionAndOrientation(
        body_id,
        piece_target_pos,
        (0.0, 0.0, 0.0, 1.0),
        physicsClientId=client_id,
    )

    # 注册手动吸附映射（不再创建 PyBullet 约束）
    # 标签盘子部件通过 JOINT_FIXED 约束跟随主 body——
    # controller 的 sync_manual_attachments 每次步进后同步位置，
    # 求解器自然会收敛标签约束链，无需额外沉降。
    RUNTIME.manually_attached_pieces[piece_id] = end_effector_id

    return OperationResult(True, f"attached {piece_id} to end effector {end_effector_id}")


def detach_piece(piece_id: str) -> OperationResult:
    """将棋子从末端执行器释放，恢复原始质量和碰撞属性。

    优先处理手动吸附（manually_attached_pieces），
    回退处理旧版约束吸附（attachment_constraints）。
    """
    if p is None:
        return OperationResult(True, f"mock detached {piece_id}")

    client_id = ensure_client()
    if client_id is None:
        return OperationResult(True, f"mock detached {piece_id}; no active client")

    # ── 优先：手动吸附解除 ──
    if piece_id in RUNTIME.manually_attached_pieces:
        del RUNTIME.manually_attached_pieces[piece_id]
        _restore_piece_dynamics(piece_id, client_id)
        return OperationResult(True, f"detached {piece_id}")

    # ── 回退：旧版约束吸附解除 ──
    constraint_id = RUNTIME.attachment_constraints.pop(piece_id, None)
    if constraint_id is not None:
        p.removeConstraint(constraint_id, physicsClientId=client_id)
        _restore_piece_dynamics(piece_id, client_id)
        return OperationResult(True, f"detached {piece_id}")

    return OperationResult(True, f"{piece_id} was not attached")


def sync_manual_attachments(
    client_id: int,
    config: Config = DEFAULT_CONFIG,
) -> None:
    """同步所有手动吸附棋子的位置到 EE 吸盘尖端。

    在每个 stepSimulation() 调用后执行，强制将棋子 teleport 到吸盘尖端正下方，
    消除约束方案中因求解器迭代不足导致的 3-120mm 间隙。

    原理：
    1. 获取 EE 连杆世界姿态
    2. 计算吸盘尖端世界坐标（EE 原点 + R_ee * (0,0,suction_cup_length)）
    3. Teleport 棋子中心到尖端下方 piece_height/2

    标签盘子部件通过自身的 JOINT_FIXED 约束跟随主 body。
    """
    if not RUNTIME.manually_attached_pieces:
        return
    robot_id = RUNTIME.robot_id
    if robot_id is None:
        return
    for piece_id, ee_id in list(RUNTIME.manually_attached_pieces.items()):
        body_id = RUNTIME.piece_body_ids.get(piece_id)
        if body_id is None:
            continue
        ee_state = p.getLinkState(robot_id, ee_id, physicsClientId=client_id)
        if ee_state is None:
            continue
        pad_tip = _transform_point(
            (0.0, 0.0, config.suction_cup_length), ee_state[0], ee_state[1]
        )
        piece_pos = (
            pad_tip[0],
            pad_tip[1],
            pad_tip[2] - config.piece_height / 2.0,
        )
        p.resetBasePositionAndOrientation(
            body_id,
            piece_pos,
            (0.0, 0.0, 0.0, 1.0),
            physicsClientId=client_id,
        )


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


def _transform_point(
    local_xyz: tuple[float, float, float],
    origin_xyz: tuple[float, float, float],
    origin_quat: tuple[float, float, float, float],
) -> tuple[float, float, float]:
    """将连杆局部坐标点转换为世界坐标。

    使用 getMatrixFromQuaternion 获取旋转矩阵，避免对 PyBullet
    内部四元数乘法约定的依赖。
    """
    R = p.getMatrixFromQuaternion(origin_quat)  # 9 元素行优先
    lx, ly, lz = local_xyz
    wx = R[0] * lx + R[1] * ly + R[2] * lz
    wy = R[3] * lx + R[4] * ly + R[5] * lz
    wz = R[6] * lx + R[7] * ly + R[8] * lz
    return (origin_xyz[0] + wx, origin_xyz[1] + wy, origin_xyz[2] + wz)
