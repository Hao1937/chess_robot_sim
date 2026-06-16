from __future__ import annotations

import math
import random
import time
from typing import Callable

from src.common.config import DEFAULT_CONFIG, Config
from src.common.types import ExecutionResult, JointTrajectory
from src.control.fk_solver import solve_fk
from src.simulation._runtime import RUNTIME, p
from src.simulation.attachment import sync_manual_attachments

# Reproducible noise seed so demos look stable across runs
_RANDOM = random.Random(42)

# ── 流式执行参数 ──
# 核心模型：恒定速度跟踪。每个 waypoint 的步数 = 关节距离 / 每步预算，
# 使运动速度恒定、与 waypoint 疏密无关：
#   - 密集插值的中间 waypoint（~0.05rad）→ 少步 → 连续流过，不停顿；
#   - 大间隙（如 home→首个 approach，可达 3+rad）→ 多步 → 平滑总体运动；
#   - 每段末 waypoint → 至少沉降 _SETTLE_STEPS，保证 attach/detach 精度。
# 这避免了「逐点固定 80/160 步沉降」模型 94% 仿真步原地空等导致的肉眼卡顿，
# 也避免了「固定少步」无法跨越大间隙（首段飞不到位）的问题。
_STREAM_STEPS_MIN = 4      # 每个中间 waypoint 的最小步数
_STREAM_STEPS_CAP = 240    # 单个 waypoint 步数上限（防止极端间隙卡死）
_SETTLE_STEPS = 150        # 每段末 waypoint 的沉降步数上限（条件式提前退出）
_SETTLE_TOLERANCE = 0.004  # 沉降收敛阈值（rad）：所有关节误差 < 此值即提前结束
_MAX_VELOCITY_FAST = 4.0
_MAX_VELOCITY_SAFE = 2.5
_SIMULATION_STEP_SECONDS = 1.0 / 240.0
# 每步关节预算（rad/step）：取 maxVelocity 的一个保守比例 / 240Hz，
# 让 PD 在该步数内能实际跟上恒定速度的 target。
_STEP_BUDGET_FRACTION = 0.6
_POSITION_GAIN = 0.6
_VELOCITY_GAIN = 1.0
_MOTOR_FORCE = 800
_EE_TRAJECTORY_COLOR = (1.0, 0.45, 0.0)
_EE_TRAJECTORY_LINE_WIDTH = 2.5

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
    # 不再 teleport 初始化：轨迹首点已由规划层从机器人**当前实际姿态**播种
    # （见 trajectory_planner.current_joint_seed），机器人从当前位置平滑驱动
    # 到首 waypoint，无瞬移。
    _pyb = _get_pybullet_context()

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
            # ── real PyBullet joint control（流式：中间少步，段末沉降）──
            is_last = (i == len(desired) - 1)
            actual_waypoint, step_time = _execute_pybullet_step(
                _pyb, waypoint, mode, is_last,
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
    is_last: bool,
) -> tuple[tuple[float, ...], float]:
    """流式驱动关节朝 *waypoint* 前进，返回实际关节角与本步耗时。

    流式模型（取代逐点沉降）：
    - 中间 waypoint：只步进少量步（fast 5 / safe 8），机器人连续流过不停顿；
      因 waypoint 已被密集插值（0.03m），少步即可平滑过渡。
    - 段末 waypoint（is_last）：沉降足够步数让 PD 收敛，保证 attach/detach 精度。

    相比旧的「固定 target + 80/160 步」逐点沉降（94% 仿真步原地空等 → 肉眼卡顿），
    流式执行消除空等、运动连续，且段末仍精确沉降。
    """
    # Clamp target joint angles to UR5 limits
    clamped = tuple(
        _clamp_joint(waypoint[j], j) for j in range(len(waypoint))
    )

    max_velocity = _MAX_VELOCITY_FAST if mode == "fast" else _MAX_VELOCITY_SAFE

    # ── 自适应步数：按当前实际位置到目标的关节距离分配 ──
    current = tuple(
        p.getJointState(ctx.robot_id, j, physicsClientId=ctx.client_id)[0]
        for j in ctx.joint_indices
    )
    max_delta = max(abs(clamped[k] - current[k]) for k in range(len(clamped)))
    step_budget = max_velocity * _STEP_BUDGET_FRACTION / 240.0
    adaptive = int(math.ceil(max_delta / step_budget)) if step_budget > 0 else _STREAM_STEPS_MIN
    if is_last:
        # 段末：取自适应步数与沉降步数的较大者，保证精度
        sim_steps = min(_STREAM_STEPS_CAP, max(adaptive, _SETTLE_STEPS))
    else:
        sim_steps = min(_STREAM_STEPS_CAP, max(adaptive, _STREAM_STEPS_MIN))

    # Apply position control to all joints（目标设定一次，随后连续步进）
    for idx, joint_idx in enumerate(ctx.joint_indices):
        p.setJointMotorControl2(
            bodyUniqueId=ctx.robot_id,
            jointIndex=joint_idx,
            controlMode=p.POSITION_CONTROL,
            targetPosition=clamped[idx],
            targetVelocity=0.0,
            force=_MOTOR_FORCE,
            maxVelocity=max_velocity,
            positionGain=_POSITION_GAIN,
            velocityGain=_VELOCITY_GAIN,
            physicsClientId=ctx.client_id,
        )

    previous_ee_position = _read_end_effector_position(ctx)
    executed = 0
    for i in range(sim_steps):
        step_started_at = time.perf_counter()
        p.stepSimulation(ctx.client_id)
        sync_manual_attachments(ctx.client_id)
        current_ee_position = _read_end_effector_position(ctx)
        _draw_end_effector_trajectory_segment(
            ctx, previous_ee_position, current_ee_position,
        )
        previous_ee_position = current_ee_position
        executed += 1
        # 每 20 步泵送一次 GUI，避免长时间阻塞导致窗口无响应
        if _pump_callback is not None and i % 20 == 0:
            _pump_callback()
        _pace_pybullet_realtime_step(step_started_at)
        # 段末沉降：收敛即提前退出，避免到位后原地空等（消除卡顿与浪费）
        if is_last and i >= _STREAM_STEPS_MIN:
            cur = tuple(
                p.getJointState(ctx.robot_id, j, physicsClientId=ctx.client_id)[0]
                for j in ctx.joint_indices
            )
            if all(abs(cur[k] - clamped[k]) < _SETTLE_TOLERANCE for k in range(len(clamped))):
                break
    sim_time = executed * _SIMULATION_STEP_SECONDS

    # Read back actual joint positions
    actual = tuple(
        p.getJointState(ctx.robot_id, j, physicsClientId=ctx.client_id)[0]
        for j in ctx.joint_indices
    )
    return actual, sim_time


def _pace_pybullet_realtime_step(step_started_at: float) -> None:
    """Keep visible PyBullet motion close to the simulated 240 Hz clock."""
    remaining = _SIMULATION_STEP_SECONDS - (time.perf_counter() - step_started_at)
    if remaining > 0.0:
        time.sleep(remaining)


def _read_end_effector_position(
    ctx: _PyBulletContext,
) -> tuple[float, float, float] | None:
    """Read the current EE world position from PyBullet."""
    ee_id = RUNTIME.end_effector_id
    if ee_id is None:
        return None
    try:
        state = p.getLinkState(
            ctx.robot_id,
            ee_id,
            physicsClientId=ctx.client_id,
        )
    except Exception:
        return None
    if state is None:
        return None
    pos = state[0]
    return (float(pos[0]), float(pos[1]), float(pos[2]))


def _draw_end_effector_trajectory_segment(
    ctx: _PyBulletContext,
    start: tuple[float, float, float] | None,
    end: tuple[float, float, float] | None,
) -> None:
    """Draw one visible segment of the actual EE trajectory."""
    if start is None or end is None:
        return
    if math.dist(start, end) < 1e-7:
        return
    try:
        debug_id = p.addUserDebugLine(
            start,
            end,
            lineColorRGB=_EE_TRAJECTORY_COLOR,
            lineWidth=_EE_TRAJECTORY_LINE_WIDTH,
            lifeTime=0,
            physicsClientId=ctx.client_id,
        )
    except Exception:
        return
    RUNTIME.ee_trajectory_debug_ids.append(debug_id)


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
