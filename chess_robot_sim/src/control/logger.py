from __future__ import annotations

from src.common.types import ExecutionResult


def summarize_execution(execution: ExecutionResult) -> dict[str, float | bool]:
    """Create a compact summary for CLI output, plots, and PPT tables."""
    return {
        "success": execution.success,
        "max_joint_error": max(execution.joint_errors, default=0.0),
        "max_end_effector_error": max(execution.end_effector_errors, default=0.0),
        "min_obstacle_clearance": min(execution.obstacle_clearances, default=0.0),
        "execution_time": execution.execution_time,
    }
