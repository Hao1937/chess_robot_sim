from __future__ import annotations

from src.common.types import ExecutionResult


def build_plot_data(execution: ExecutionResult) -> dict[str, list[float]]:
    """Return plot-ready data without requiring matplotlib during tests."""
    return {
        "joint_error": execution.joint_errors,
        "end_effector_error": execution.end_effector_errors,
        "obstacle_clearance": execution.obstacle_clearances,
    }
