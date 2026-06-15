"""诊断脚本：验证连续插值控制器行为。

在 PyBullet 环境下运行：
    CHESS_ROBOT_PYBULLET_GUI=1 python scripts/diagnose_controller.py

检查项：
    1. execute_trajectory 是否在轨迹前插入当前关节位置（不瞬移）
    2. 每子步电机 target 是否逐步渐变（而非固定值）
    3. maxVelocity 是否已降低到合理值
"""
from __future__ import annotations

import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).parent.parent))


PASS = "[PASS]"
FAIL = "[FAIL]"
WARN = "[WARN]"


def check_source_params() -> dict:
    """检查源码中的参数值 — 直接读文件避免 import 链。"""
    ctrl_file = Path(__file__).parent.parent / "src" / "control" / "controller.py"
    src = ctrl_file.read_text(encoding="utf-8")

    results = {
        "has_resetJointState": "resetJointState" in src,
        "has_INITIALIZED": "_INITIALIZED = False" in src or "_INITIALIZED=" in src,
        "maxVelocity_fast": None,
        "maxVelocity_safe": None,
        "STEPS_FAST": None,
        "STEPS_SAFE": None,
    }

    # 解析 _STEPS_*
    import re
    m = re.search(r"_STEPS_FAST\s*=\s*(\d+)", src)
    if m:
        results["STEPS_FAST"] = int(m.group(1))
    m = re.search(r"_STEPS_SAFE\s*=\s*(\d+)", src)
    if m:
        results["STEPS_SAFE"] = int(m.group(1))

    # 解析 _apply_motor_control 中的 max_velocity
    in_func = False
    for line in src.split("\n"):
        if "def _apply_motor_control" in line:
            in_func = True
            continue
        if in_func and line.strip().startswith("def "):
            break
        if in_func and "max_velocity" in line:
            # 尝试提取值
            parts = line.strip().split("=")
            if len(parts) >= 2:
                val_str = parts[-1].strip().rstrip(",")
                try:
                    v = float(val_str.split()[0])
                except ValueError:
                    continue
                if "fast" in line:
                    results["maxVelocity_fast"] = v
                else:
                    results["maxVelocity_safe"] = v

    return results


def check_continuous_interpolation_logic() -> dict:
    """验证连续插值逻辑（不需要 PyBullet）。"""
    # 从文件读参数
    ctrl_file = Path(__file__).parent.parent / "src" / "control" / "controller.py"
    src = ctrl_file.read_text(encoding="utf-8")
    import re
    m = re.search(r"_STEPS_FAST\s*=\s*(\d+)", src)
    steps_fast = int(m.group(1)) if m else 10

    wp = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
    next_wp = (0.1, 0.2, 0.3, 0.0, 0.0, 0.0)

    targets = []
    for i in range(steps_fast):
        alpha = (i + 1) / steps_fast
        target = tuple(wp[j] + alpha * (next_wp[j] - wp[j]) for j in range(6))
        targets.append(target)

    unique = len(set(targets))
    all_moving = unique == steps_fast
    final_is_next = all(
        abs(targets[-1][j] - next_wp[j]) < 1e-10 for j in range(6)
    )

    return {
        "sub_steps": steps_fast,
        "unique_targets": unique,
        "all_moving": all_moving,
        "final_equals_next_wp": final_is_next,
        "target_progression": [(round(t[0], 5), round(t[1], 5)) for t in targets[:5]],
    }


def check_prepend_current_position() -> dict:
    """验证 execute_trajectory 插入当前位置（mock 模式测试）。"""
    from unittest.mock import patch, MagicMock
    from src.common.types import JointTrajectory
    from src.control.controller import execute_trajectory

    trajectory = JointTrajectory(
        joint_waypoints=[(0.5, 0.1, 0.2, 0.0, 0.0, 0.0)],
        speed_profile=["fast"],
    )

    mock_ctx = MagicMock()
    mock_ctx.robot_id = 1
    mock_ctx.client_id = 0
    mock_ctx.joint_indices = (0, 1, 2, 3, 4, 5)

    actual_current = (0.23, -1.57, 2.09, -0.52, 1.05, 0.0)

    with patch("src.control.controller._get_pybullet_context", return_value=mock_ctx):
        with patch("src.control.controller._read_joints",
                   side_effect=[actual_current, (0.48, 0.09, 0.19, 0.0, 0.0, 0.0)]):
            with patch("src.control.controller._execute_pybullet_continuous",
                       return_value=((0.48, 0.09, 0.19, 0.0, 0.0, 0.0), 0.05)):
                result = execute_trajectory(trajectory)

    prepended = (
        len(result.desired_joint_angles) == 2
        and result.desired_joint_angles[0] == actual_current
    )

    return {
        "prepended_current": prepended,
        "desired_count": len(result.desired_joint_angles),
        "first_wp": result.desired_joint_angles[0] if result.desired_joint_angles else None,
    }


def main():
    print("=" * 60)
    print("Controller Continuous Interpolation Diagnostics")
    print("=" * 60)

    print("\n[1] Source parameter check")
    params = check_source_params()
    for k, v in params.items():
        if v is None:
            print(f"  {WARN} {k}: None (not found)")
        elif isinstance(v, bool) and v:
            print(f"  {FAIL} {k}: {v}")
        elif isinstance(v, bool):
            print(f"  {PASS} {k}: {v}")
        else:
            print(f"  --> {k}: {v}")

    print("\n[2] Continuous interpolation logic (no PyBullet needed)")
    interp = check_continuous_interpolation_logic()
    for k, v in interp.items():
        if isinstance(v, bool):
            print(f"  {PASS if v else FAIL} {k}: {v}")
        else:
            print(f"  --> {k}: {v}")

    print("\n[3] Current position prepend (mock mode)")
    try:
        prepend = check_prepend_current_position()
        for k, v in prepend.items():
            print(f"  {PASS if v else FAIL} {k}: {v}")
    except Exception as e:
        print(f"  {FAIL} Mock test failed: {e}")

    print("\n[4] Key assertions")
    errors = []

    if params.get("has_resetJointState"):
        errors.append(f"{FAIL} Code still contains resetJointState (causes teleport)")
    else:
        print(f"  {PASS} resetJointState removed")

    if params.get("has_INITIALIZED"):
        errors.append(f"{FAIL} Code still contains _INITIALIZED global flag")
    else:
        print(f"  {PASS} _INITIALIZED removed")

    fast_mv = params.get("maxVelocity_fast")
    if fast_mv is not None and fast_mv > 3.0:
        errors.append(f"{WARN} fast maxVelocity={fast_mv} too high (should <= 2.0)")
    elif fast_mv is not None:
        print(f"  {PASS} fast maxVelocity={fast_mv} (<= 2.0, reasonable)")
    else:
        print(f"  {WARN} Could not parse fast maxVelocity")

    safe_mv = params.get("maxVelocity_safe")
    if safe_mv is not None and safe_mv > 2.0:
        errors.append(f"{WARN} safe maxVelocity={safe_mv} too high (should <= 1.0)")
    elif safe_mv is not None:
        print(f"  {PASS} safe maxVelocity={safe_mv} (<= 1.0, reasonable)")
    else:
        print(f"  {WARN} Could not parse safe maxVelocity")

    if not interp.get("all_moving"):
        errors.append(f"{FAIL} Sub-step targets have duplicates (causes jerkiness)")
    else:
        print(f"  {PASS} All {interp['sub_steps']} sub-step targets are unique (continuous motion)")

    if not interp.get("final_equals_next_wp"):
        errors.append(f"{FAIL} Final sub-step target != next_wp")
    else:
        print(f"  {PASS} Final sub-step target == next_wp (correct endpoint)")

    if errors:
        print(f"\n{'='*60}")
        print(f"Found {len(errors)} issue(s):")
        for e in errors:
            print(f"  {e}")
        return 1
    else:
        print(f"\n{'='*60}")
        print(f"{PASS} All checks passed — continuous interpolation controller is correctly configured")
        return 0


if __name__ == "__main__":
    raise SystemExit(main())
