from __future__ import annotations

from src.common.config import DEFAULT_CONFIG, Config
from src.common.types import RobotHandle
from src.simulation._runtime import RUNTIME, ensure_client, p, project_root


def load_robot(urdf_path: str | None = None) -> RobotHandle:
    """Load the UR5 model and return the ids needed by downstream modules.

    When PyBullet is unavailable, this keeps the old mock-safe behavior so the
    interface tests and planning pipeline can still run.
    """
    if p is None:
        return RobotHandle(robot_id=1, end_effector_id=6, joint_indices=(0, 1, 2, 3, 4, 5))

    client_id = ensure_client()
    if client_id is None:
        return RobotHandle(robot_id=1, end_effector_id=6, joint_indices=(0, 1, 2, 3, 4, 5))

    model_path = urdf_path or str(project_root() / "assets" / "ur5" / "ur5_joint_limited_robot.urdf")
    base_position = Config.base_link_position
    try:
        robot_id = p.loadURDF(
            model_path,
            basePosition=base_position,
            baseOrientation=p.getQuaternionFromEuler((0.0, 0.0, 0.0)),
            useFixedBase=True,
            flags=p.URDF_USE_SELF_COLLISION,
            physicsClientId=client_id,
        )
    except Exception:
        return RobotHandle(robot_id=1, end_effector_id=6, joint_indices=(0, 1, 2, 3, 4, 5))

    joint_indices: list[int] = []
    end_effector_id = -1
    preferred_ee_names = ["tool0", "suction_cup_link", "ee_link", "wrist_3_link"]
    # tool0 优先 —— 其 COM 在 link frame origin (0,0,0)，
    # getLinkState 返回的 COM 位置 = link origin，无偏移。
    # suction_cup_link 的 COM 在 (0,0,0.02)，getLinkState 会偏移 20mm，
    # 导致吸盘尖端位置计算错误（R6 根因）。

    # 先收集所有 link 名称 → joint_index 映射
    link_name_to_index: dict[str, int] = {}
    for joint_index in range(p.getNumJoints(robot_id, physicsClientId=client_id)):
        joint_info = p.getJointInfo(robot_id, joint_index, physicsClientId=client_id)
        joint_type = joint_info[2]
        link_name = joint_info[12].decode("utf-8")
        link_name_to_index[link_name] = joint_index
        if joint_type in {p.JOINT_REVOLUTE, p.JOINT_PRISMATIC}:
            joint_indices.append(joint_index)

    # 按优先级顺序选择第一个存在的 EE link
    for name in preferred_ee_names:
        idx = link_name_to_index.get(name)
        if idx is not None:
            end_effector_id = idx
            break

    if end_effector_id < 0:
        end_effector_id = p.getNumJoints(robot_id, physicsClientId=client_id) - 1

    _apply_robot_visual_style(robot_id, client_id)
    RUNTIME.robot_id = robot_id
    RUNTIME.end_effector_id = end_effector_id
    RUNTIME.joint_indices = tuple(joint_indices)

    # ── 初始化关节到 home_pose ──
    # 消除「机器人加载后停在全零奇异位形」的问题：全零位形会让 IK 链的
    # 当前姿态种子落在奇异点，且第一条命令从全零驱动会扫过大空间。
    # 设到 home_pose 后第一条命令从已知抬高姿态平滑出发。
    home = Config.home_pose[:6]
    for idx, joint_index in enumerate(joint_indices[:6]):
        p.resetJointState(robot_id, joint_index, home[idx], physicsClientId=client_id)
        # 位置电机保持 home_pose，避免重力下垂
        p.setJointMotorControl2(
            robot_id, joint_index,
            controlMode=p.POSITION_CONTROL,
            targetPosition=home[idx],
            force=800,
            physicsClientId=client_id,
        )

    return RobotHandle(robot_id=robot_id, end_effector_id=end_effector_id, joint_indices=tuple(joint_indices))



def _apply_robot_visual_style(robot_id: int, client_id: int) -> None:
    """Give the UR5 readable industrial colors when mesh materials are absent."""
    if p is None:
        return

    link_colors = {
        "base_link": ((0.20, 0.21, 0.21, 1.0), (0.22, 0.22, 0.22)),
        "shoulder_link": ((0.35, 0.36, 0.35, 1.0), (0.20, 0.20, 0.20)),
        "upper_arm_link": ((0.74, 0.75, 0.72, 1.0), (0.18, 0.18, 0.18)),
        "forearm_link": ((0.70, 0.71, 0.68, 1.0), (0.18, 0.18, 0.18)),
        "wrist_1_link": ((0.28, 0.29, 0.28, 1.0), (0.18, 0.18, 0.18)),
        "wrist_2_link": ((0.82, 0.55, 0.20, 1.0), (0.20, 0.15, 0.08)),
        "wrist_3_link": ((0.28, 0.29, 0.28, 1.0), (0.18, 0.18, 0.18)),
        "ee_link": ((0.18, 0.18, 0.17, 1.0), (0.08, 0.08, 0.08)),
        "tool0": ((0.18, 0.18, 0.17, 1.0), (0.08, 0.08, 0.08)),
        "suction_cup_link": ((0.05, 0.05, 0.05, 1.0), (0.06, 0.06, 0.06)),
    }

    p.changeVisualShape(
        robot_id,
        -1,
        rgbaColor=link_colors["base_link"][0],
        specularColor=link_colors["base_link"][1],
        physicsClientId=client_id,
    )
    for joint_index in range(p.getNumJoints(robot_id, physicsClientId=client_id)):
        joint_info = p.getJointInfo(robot_id, joint_index, physicsClientId=client_id)
        link_name = joint_info[12].decode("utf-8")
        style = link_colors.get(link_name)
        if style is None:
            continue
        rgba_color, specular_color = style
        p.changeVisualShape(
            robot_id,
            joint_index,
            rgbaColor=rgba_color,
            specularColor=specular_color,
            physicsClientId=client_id,
        )
