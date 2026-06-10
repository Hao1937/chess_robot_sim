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
    base_position = _side_middle_base_position(DEFAULT_CONFIG)
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
    preferred_ee_names = {"suction_cup_link", "tool0", "ee_link", "wrist_3_link"}
    for joint_index in range(p.getNumJoints(robot_id, physicsClientId=client_id)):
        joint_info = p.getJointInfo(robot_id, joint_index, physicsClientId=client_id)
        joint_type = joint_info[2]
        link_name = joint_info[12].decode("utf-8")
        if joint_type in {p.JOINT_REVOLUTE, p.JOINT_PRISMATIC}:
            joint_indices.append(joint_index)
        if link_name in preferred_ee_names:
            end_effector_id = joint_index

    if end_effector_id < 0:
        end_effector_id = p.getNumJoints(robot_id, physicsClientId=client_id) - 1

    RUNTIME.robot_id = robot_id
    RUNTIME.end_effector_id = end_effector_id
    RUNTIME.joint_indices = tuple(joint_indices)
    return RobotHandle(robot_id=robot_id, end_effector_id=end_effector_id, joint_indices=tuple(joint_indices))


def _side_middle_base_position(config: Config) -> tuple[float, float, float]:
    """Place the robot base near the middle of the board's left side."""
    x0, y0, _ = config.board_origin
    board_mid_y = y0 + (config.board_rows - 1) * config.cell_size / 2.0
    side_offset = 5.0 * config.cell_size
    return (x0 - side_offset, board_mid_y, config.z_board)
