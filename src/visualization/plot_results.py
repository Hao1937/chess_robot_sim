from __future__ import annotations

import csv
from pathlib import Path

from src.common.types import ExecutionResult


def build_plot_data(execution: ExecutionResult) -> dict[str, list[float]]:
    """Return plot-ready data without requiring matplotlib during tests."""
    return {
        "joint_error": execution.joint_errors,
        "end_effector_error": execution.end_effector_errors,
        "obstacle_clearance": execution.obstacle_clearances,
    }


def save_plots(
    execution: ExecutionResult,
    output_dir: str = "results",
    label: str = "",
) -> list[str]:
    """Save joint error, EE error, and clearance plots as PNG files.

    Returns the list of saved file paths.
    """
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    out = Path(output_dir)
    out.mkdir(parents=True, exist_ok=True)
    prefix = f"{label}_" if label else ""

    data = build_plot_data(execution)
    saved: list[str] = []

    configs = [
        ("joint_error", "Joint Tracking Error (rad)", "red"),
        ("end_effector_error", "End-Effector Position Error (m)", "blue"),
        ("obstacle_clearance", "Obstacle Clearance (m)", "green"),
    ]
    for key, ylabel, color in configs:
        values = data.get(key, [])
        if not values:
            continue
        fig, ax = plt.subplots(figsize=(10, 4))
        ax.plot(values, color=color, linewidth=1.5)
        ax.set_xlabel("Waypoint Index")
        ax.set_ylabel(ylabel)
        ax.set_title(ylabel)
        ax.grid(True, alpha=0.3)

        path = out / f"{prefix}{key}.png"
        fig.savefig(str(path), dpi=150, bbox_inches="tight")
        plt.close(fig)
        saved.append(str(path))

    return saved


def save_execution_csv(
    execution: ExecutionResult,
    output_dir: str = "results",
    label: str = "",
) -> str:
    """Export desired/actual/error per waypoint to a CSV file.

    Returns the saved file path.
    """
    out = Path(output_dir)
    out.mkdir(parents=True, exist_ok=True)
    prefix = f"{label}_" if label else ""
    path = out / f"{prefix}execution.csv"

    n = len(execution.desired_joint_angles)
    rows: list[dict[str, object]] = []
    for i in range(n):
        row: dict[str, object] = {"waypoint": i}
        for j, (d, a) in enumerate(
            zip(execution.desired_joint_angles[i], execution.actual_joint_angles[i])
        ):
            row[f"joint{j}_desired"] = d
            row[f"joint{j}_actual"] = a
        row["joint_error"] = execution.joint_errors[i] if i < len(execution.joint_errors) else None
        row["ee_error"] = execution.end_effector_errors[i] if i < len(execution.end_effector_errors) else None
        row["clearance"] = execution.obstacle_clearances[i] if i < len(execution.obstacle_clearances) else None
        rows.append(row)

    if not rows:
        return str(path)

    with open(path, "w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)

    return str(path)
