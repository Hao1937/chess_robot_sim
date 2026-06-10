from __future__ import annotations

import random

from src.common.types import ExecutionResult, JointTrajectory

# Reproducible "noise" seed so demos look stable across runs
_RANDOM = random.Random(42)


def execute_trajectory(
    trajectory: JointTrajectory,
    *,
    joint_noise_std: float = 0.002,
    ee_noise_std: float = 0.003,
    fast_step_time: float = 0.15,
    safe_step_time: float = 0.45,
) -> ExecutionResult:
    """Execute a joint trajectory with simulated tracking errors.

    The skeleton simulates the robot following desired waypoints while injecting
    small per-joint Gaussian noise and a slight cumulative drift so that the
    error grows over long trajectories — similar to a real position-controlled
    arm without perfect feed-forward.

    When B connects the real PyBullet robot, replace the body of this function
    with a joint-space position-control / PID loop while keeping the same
    return type.
    """
    desired = trajectory.joint_waypoints
    if not desired:
        return ExecutionResult(
            success=True,
            desired_joint_angles=[],
            actual_joint_angles=[],
            joint_errors=[],
            end_effector_errors=[],
            obstacle_clearances=[],
            execution_time=0.0,
        )

    actual: list[tuple[float, ...]] = []
    joint_errors: list[float] = []
    ee_errors: list[float] = []
    clearances: list[float] = []
    total_time = 0.0
    cumulative_drift = [0.0] * len(desired[0])

    for i, waypoint in enumerate(desired):
        # cumulative drift grows slowly (brownian-like)
        cumulative_drift = [d + _RANDOM.gauss(0, joint_noise_std * 0.5) for d in cumulative_drift]

        actual_waypoint = tuple(
            value + _RANDOM.gauss(0, joint_noise_std) + cumulative_drift[j]
            for j, value in enumerate(waypoint)
        )
        actual.append(actual_waypoint)

        # joint error: RMS across this waypoint's joints
        squared = [(actual_waypoint[j] - waypoint[j]) ** 2 for j in range(len(waypoint))]
        joint_errors.append((sum(squared) / len(squared)) ** 0.5)

        # end-effector error: scaled proxy for Cartesian error
        base_ee = joint_errors[-1] * 1.6 + abs(_RANDOM.gauss(0, ee_noise_std))
        ee_errors.append(round(base_ee, 6))

        # obstacle clearance: tight when safe, generous when fast + noise
        mode = trajectory.speed_profile[i] if i < len(trajectory.speed_profile) else "safe"
        if mode == "safe":
            clearance = 0.015 + abs(_RANDOM.gauss(0, 0.008))
        else:
            clearance = 0.05 + abs(_RANDOM.gauss(0, 0.02))
        clearances.append(round(clearance, 4))

        step_time = safe_step_time if mode == "safe" else fast_step_time
        total_time += step_time

    return ExecutionResult(
        success=True,
        desired_joint_angles=desired,
        actual_joint_angles=actual,
        joint_errors=joint_errors,
        end_effector_errors=ee_errors,
        obstacle_clearances=clearances,
        execution_time=round(total_time, 3),
    )
