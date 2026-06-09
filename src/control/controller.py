from __future__ import annotations

from src.common.types import ExecutionResult, JointTrajectory


def execute_trajectory(trajectory: JointTrajectory) -> ExecutionResult:
    """Execute a joint trajectory.

    The skeleton mirrors desired angles as actual angles with small mock errors.
    D can replace this with PyBullet position control or PID.
    """
    desired = trajectory.joint_waypoints
    actual = [tuple(value + 0.001 for value in waypoint) for waypoint in desired]
    joint_errors = [0.001 * len(waypoint) for waypoint in desired]
    ee_errors = [0.002 for _ in desired]
    clearances = [0.05 if mode == "fast" else 0.03 for mode in trajectory.speed_profile]
    return ExecutionResult(
        success=True,
        desired_joint_angles=desired,
        actual_joint_angles=actual,
        joint_errors=joint_errors,
        end_effector_errors=ee_errors,
        obstacle_clearances=clearances,
        execution_time=max(0.1, 0.2 * len(desired)),
    )
