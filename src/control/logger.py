from __future__ import annotations

import csv
from pathlib import Path

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


def save_summary_csv(
    summaries: list[dict[str, object]],
    output_path: str = "results/summary_table.csv",
) -> str:
    """Save a list of execution summaries as a CSV table (one row per scenario).

    Each summary dict can contain extra metadata keys such as ``scenario``,
    ``obstacle_mode``, ``hand_on``, ``safety_decision``, etc.
    """
    out = Path(output_path)
    out.parent.mkdir(parents=True, exist_ok=True)

    if not summaries:
        # Write an empty file with a header so the file always exists.
        with open(out, "w", newline="", encoding="utf-8") as f:
            f.write("scenario\n")
        return str(out)

    fieldnames = list(summaries[0].keys())
    with open(out, "w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(summaries)

    return str(out)
