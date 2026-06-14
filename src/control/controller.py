from __future__ import annotations

import math
import random
from typing import Callable

from src.common.config import DEFAULT_CONFIG, Config
from src.common.types import ExecutionResult, JointTrajectory
from src.control.fk_solver import solve_fk
from src.simulation._runtime import RUNTIME, p
from src.simulation.attachment import sync_manual_attachments

# Reproducible noise seed so demos look stable across runs
_RANDOM = random.Random(42)

# 标志：确保机械臂关节只初始化一次（避免每次 trajectory 段都 teleport）
_INITIALIZED = False

# 可选回调：在每个 waypoint 仿真步进后调用，用于保持 GUI 事件循环活跃
_pump_callback: Callable[[], None] | None = None


def set_simulation_pump_callback(callback: Callable[[], None] | None) -> None:
    """设置仿真步进期间的回调（如 matplotlib GUI 事件泵送）。

    传 None 关闭回调。仅在交互模式下使用。
    """
    global _pump_callback
    _pump_callback = callback

# UR5 joint limits (radians) — from the URDF joint_limited model
_JOINT_LIMITS = [
    (-2.0 * math.pi, 2.0 * math.pi),   # shoulder_pan
    (-2.0 * math.pi, 2.0 * math.pi),   # shoulder_lift
    (-2.0 * math.pi, 2.0 * math.pi),   # elbow
    (-2.0 * math.pi, 2.0 * math.pi),   # wrist_1
    (-2.0 * math.pi, 2.0 * math.pi),   # wrist_2
    (-2.0 * math.pi, 2.0 * math.pi),   # wrist_3
]


def execute_trajectory(
    trajectory: JointTrajectory,
    *,
    joint_noise_std: float = 0.002,
    ee_noise_std: float = 0.003,
    fast_step_time: float = 0.15,
    safe_step_time: float = 0.45,
    config: Config = DEFAULT_CONFIG,
) -> ExecutionResult:
    """Execute a joint trajectory with simulated tracking errors.

    The skeleton simulates the robot following desired waypoints while:
    - Injecting per-joint Gaussian noise proportional to step magnitude
    - Adding slow cumulative drift (brownian-like)
    - Computing real end-effector error via FK (not proxy scaling)
    - Clamping actual joint angles to UR5 limits
    - Simulating obstacle clearance tied to speed mode

    When B connects the real PyBullet robot, replace this function's body
    with a joint-space position-control / PID loop while keeping the same
    return type.

    Args:
        trajectory: JointTrajectory with joint_waypoints and speed_profile
        joint_noise_std: per-joint Gaussian noise standard deviation (rad)
        ee_noise_std: additional EE drift noise (unused when FK is active)
        fast_step_time: time per waypoint in fast mode (s)
        safe_step_time: time per waypoint in safe mode (s)
        config: configuration (used for FK base_link_position)

    Returns:
        ExecutionResult with desired/actual angles, FK-based EE errors, etc.
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

    # ── PyBullet integration ──
    _pyb = _get_pybullet_context()

    # 初始化：仅在首次调用时将机械臂关节重置到轨迹起点
    # 后续调用跳过，因为上一段轨迹的终点就是下一段的起点
    global _INITIALIZED
    if _pyb is not None and desired and not _INITIALIZED:
        _init_joint_state(_pyb, desired[0])
        _INITIALIZED = True

    actual: list[tuple[float, ...]] = []
    joint_errors: list[float] = []
    ee_errors: list[float] = []
    clearances: list[float] = []
    total_time = 0.0
    cumulative_drift = [0.0] * len(desired[0])

    for i, waypoint in enumerate(desired):
        mode = (
            trajectory.speed_profile[i]
            if i < len(trajectory.speed_profile)
            else "safe"
        )

        if _pyb is not None:
            # ── real PyBullet joint control ──
            actual_waypoint, step_time = _execute_pybullet_step(
                _pyb, waypoint, mode, fast_step_time, safe_step_time,
            )
        else:
            # ── mock noise model ──
            actual_waypoint, step_time = _execute_mock_step(
                waypoint, i, mode, desired,
                joint_noise_std, cumulative_drift,
                fast_step_time, safe_step_time,
            )

        actual.append(actual_waypoint)

        # ── joint error: RMS across this waypoint's 6 joints ──
        squared = [
            (actual_waypoint[j] - waypoint[j]) ** 2
            for j in range(len(waypoint))
        ]
        joint_errors.append((sum(squared) / len(squared)) ** 0.5)

        # ── end-effector error: real FK-based position error ──
        ee_error = _compute_fk_ee_error(waypoint, actual_waypoint, config)
        ee_errors.append(round(ee_error, 6))

        # ── obstacle clearance ──
        if _pyb is not None:
            # 实际仿真中障碍物距离通过仿真物理自动处理，
            # 此处根据速度模式给出合理估算值
            if mode == "safe":
                clearance = 0.015
            else:
                clearance = 0.05
        else:
            if mode == "safe":
                clearance = 0.015 + abs(_RANDOM.gauss(0, 0.008))
            else:
                clearance = 0.05 + abs(_RANDOM.gauss(0, 0.02))
        clearances.append(round(clearance, 4))

        # ── timing ──
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


# ── PyBullet context & step helpers ──


class _PyBulletContext:
    """Minimal handle for PyBullet joint control."""
    __slots__ = ("robot_id", "client_id", "joint_indices")
    robot_id: int
    client_id: int
    joint_indices: tuple[int, ...]


def _init_joint_state(ctx: _PyBulletContext, waypoint: tuple[float, ...]) -> None:
    """快速将机械臂传送到轨迹起点，沉降期间暂停棋子碰撞。

    策略（经过 5 轮迭代确定的稳定方案）：
    1. 临时将所有场景物体设为静态（mass=0），防止机械臂传送后
       物理引擎因穿透施加分离力导致棋子炸飞
    2. 用 resetJointState 瞬间传送关节（快速，不扫过空间）
    3. 运行沉降步数让 PD 控制器稳定
    4. 恢复场景物体质量
    """
    clamped = tuple(
        _clamp_joint(waypoint[j], j) for j in range(len(waypoint))
    )

    # ── 临时冻结场景物体（mass=0），防止传送后穿透力炸飞棋子 ──
    saved_masses: dict[int, float] = {}
    for body_id in RUNTIME.scene_body_ids:
        try:
            dyn = p.getDynamicsInfo(body_id, -1, physicsClientId=ctx.client_id)
            saved_masses[body_id] = dyn[0]
            if dyn[0] > 0:
                p.changeDynamics(body_id, -1, mass=0.0, physicsClientId=ctx.client_id)
        except Exception:
            pass

    try:
        # 设置电机目标并瞬间传送关节
        for idx, joint_idx in enumerate(ctx.joint_indices):
            p.setJointMotorControl2(
                bodyUniqueId=ctx.robot_id,
                jointIndex=joint_idx,
                controlMode=p.POSITION_CONTROL,
                targetPosition=clamped[idx],
                targetVelocity=0.0,
                force=1000,
                maxVelocity=3.0,
                positionGain=1.2,
                velocityGain=0.8,
                physicsClientId=ctx.client_id,
            )
        for idx, joint_idx in enumerate(ctx.joint_indices):
            p.resetJointState(
                ctx.robot_id, joint_idx,
                targetValue=clamped[idx],
                physicsClientId=ctx.client_id,
            )

        # 沉降步数（棋子已冻结，穿透不会产生力，步数可以减少）
        for i in range(60):
            p.stepSimulation(ctx.client_id)
            sync_manual_attachments(ctx.client_id)
            if _pump_callback is not None and i % 15 == 0:
                _pump_callback()
    finally:
        # ── 恢复场景物体质量 ──
        for body_id, mass in saved_masses.items():
            if mass > 0:
                try:
                    p.changeDynamics(body_id, -1, mass=mass, physicsClientId=ctx.client_id)
                except Exception:
                    pass


def _get_pybullet_context() -> _PyBulletContext | None:
    """Return a PyBullet context when the simulation is connected and ready."""
    if p is None:
        return None
    robot_id = RUNTIME.robot_id
    client_id = RUNTIME.client_id
    joint_indices = RUNTIME.joint_indices
    if robot_id is None or client_id is None or not joint_indices:
        return None
    if not p.isConnected(client_id):
        return None
    ctx = _PyBulletContext()
    ctx.robot_id = robot_id
    ctx.client_id = client_id
    ctx.joint_indices = joint_indices
    return ctx


def _execute_pybullet_step(
    ctx: _PyBulletContext,
    waypoint: tuple[float, ...],
    mode: str,
    fast_step_time: float,
    safe_step_time: float,
) -> tuple[tuple[float, ...], float]:
    """Drive PyBullet joints toward *waypoint* and return actual joint angles + elapsed sim time."""
    # Clamp target joint angles to UR5 limits
    clamped = tuple(
        _clamp_joint(waypoint[j], j) for j in range(len(waypoint))
    )

    # 更高的增益和力矩以保证机械臂在重力下稳定跟踪轨迹
    max_force = 1000 if mode == "fast" else 600
    max_velocity = 10.0 if mode == "fast" else 5.0
    position_gain = 1.2
    velocity_gain = 0.8

    # Apply position control to all joints
    for idx, joint_idx in enumerate(ctx.joint_indices):
        p.setJointMotorControl2(
            bodyUniqueId=ctx.robot_id,
            jointIndex=joint_idx,
            controlMode=p.POSITION_CONTROL,
            targetPosition=clamped[idx],
            targetVelocity=0.0,
            force=max_force,
            maxVelocity=max_velocity,
            positionGain=position_gain,
            velocityGain=velocity_gain,
            physicsClientId=ctx.client_id,
        )

    # Step simulation to let joints settle toward targets
    # 每 waypoint 增加步数（fast: 80, safe: 160），确保 PD 控制器能跟踪到位
    sim_steps = 80 if mode == "fast" else 160
    for i in range(sim_steps):
        p.stepSimulation(ctx.client_id)
        sync_manual_attachments(ctx.client_id)
        # 每 20 步泵送一次 GUI，避免长时间阻塞导致窗口无响应
        if _pump_callback is not None and i % 20 == 0:
            _pump_callback()
    sim_time = sim_steps * (1.0 / 240)

    # Read back actual joint positions
    actual = tuple(
        p.getJointState(ctx.robot_id, j, physicsClientId=ctx.client_id)[0]
        for j in ctx.joint_indices
    )
    return actual, sim_time


def _execute_mock_step(
    waypoint: tuple[float, ...],
    step_index: int,
    mode: str,
    all_waypoints: list[tuple[float, ...]],
    joint_noise_std: float,
    cumulative_drift: list[float],
    fast_step_time: float,
    safe_step_time: float,
) -> tuple[tuple[float, ...], float]:
    """Simulate tracking errors with Gaussian noise + brownian drift (original behaviour)."""
    # Per-joint Gaussian noise
    joint_noise = [
        _RANDOM.gauss(0, joint_noise_std) for _ in range(len(waypoint))
    ]

    # Cumulative brownian drift
    for d_idx in range(len(cumulative_drift)):
        cumulative_drift[d_idx] += _RANDOM.gauss(0, joint_noise_std * 0.3)

    actual_waypoint = tuple(
        _clamp_joint(
            waypoint[j] + joint_noise[j] + cumulative_drift[j],
            j,
        )
        for j in range(len(waypoint))
    )

    step_time = safe_step_time if mode == "safe" else fast_step_time
    return actual_waypoint, step_time


# ── internal helpers ──


def _compute_fk_ee_error(
    desired_joint: tuple[float, ...],
    actual_joint: tuple[float, ...],
    config: Config,
) -> float:
    """Compute Euclidean end-effector position error using FK."""
    desired_xyz = solve_fk(desired_joint, config)
    actual_xyz = solve_fk(actual_joint, config)
    return math.hypot(
        desired_xyz[0] - actual_xyz[0],
        desired_xyz[1] - actual_xyz[1],
        desired_xyz[2] - actual_xyz[2],
    )


def _clamp_joint(value: float, joint_index: int) -> float:
    """Clamp a joint angle to its UR5 limit range."""
    lo, hi = _JOINT_LIMITS[joint_index]
    return min(hi, max(lo, value))


def reset_initialization() -> None:
    """重置关节初始化标志，使下次 execute_trajectory 重新初始化关节。

    在场景重新加载或手动复位机械臂后调用。
    """
    global _INITIALIZED
    _INITIALIZED = False
