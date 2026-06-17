#!/usr/bin/env python3
"""Collect presentation metrics from the interactive chess-robot pipeline.

The script feeds a fixed command sequence into the same run_interactive() entry
point used by ``python main.py --interactive``. It forces PyBullet DIRECT mode
and the no-op board GUI so the run is reproducible on headless machines.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import os
import sys
import time
from pathlib import Path
from typing import Iterable


PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT))

# Set these before importing modules that create the PyBullet runtime.
os.environ["CHESS_ROBOT_PYBULLET_GUI"] = "0"
os.environ["CHESS_ROBOT_BOARD_GUI"] = "0"
os.environ.setdefault("MPLBACKEND", "Agg")

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.collections import LineCollection
from matplotlib.lines import Line2D
from matplotlib.patches import Circle, FancyArrowPatch, Rectangle

from main import run_interactive
import main as main_module
from src.common.config import DEFAULT_CONFIG, Config
from src.common.types import ExecutionResult, LogicalAction, MoveCommand
from src.control.fk_solver import solve_fk
from src.control.logger import summarize_execution
from src.simulation._runtime import RUNTIME, p, pybullet_available
from src.visualization.plot_results import save_execution_csv, save_plots


COMMANDS = [
    "A1 A3",
    "B10 A8",
    "H3 H10",
    "C10 E8",
    "A8 B6",
    "B6 A4",
    "reset",
]

PALETTE = [
    "#2563eb",
    "#dc2626",
    "#059669",
    "#7c3aed",
    "#ea580c",
    "#0891b2",
    "#4b5563",
]


class CommandFeeder:
    def __init__(self, commands: Iterable[str]) -> None:
        self.commands = list(commands)
        self.index = 0

    def __call__(self, prompt: str = "") -> str:
        if self.index >= len(self.commands):
            return "quit"
        command = self.commands[self.index]
        self.index += 1
        print(f"[interactive input] {command}")
        return command


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Collect headless PyBullet metrics for presentation figures."
    )
    parser.add_argument(
        "--output",
        default="results/presentation_interactive_metrics",
        help="Output directory for CSV, PNG, and manifest files.",
    )
    parser.add_argument(
        "--commands",
        nargs="*",
        default=COMMANDS,
        help="Interactive commands to feed. Defaults to the presentation sequence.",
    )
    args = parser.parse_args()

    if not pybullet_available():
        raise RuntimeError("pybullet is required for this collection script")

    output_dir = Path(args.output)
    output_dir.mkdir(parents=True, exist_ok=True)

    timings: list[float] = []
    original_run_command = main_module.run_command

    def timed_run_command(*run_args, **run_kwargs):
        started = time.perf_counter()
        result = original_run_command(*run_args, **run_kwargs)
        result["wall_time_s"] = round(time.perf_counter() - started, 4)
        timings.append(result["wall_time_s"])
        return result

    main_module.run_command = timed_run_command
    try:
        session = run_interactive(
            input_func=CommandFeeder(args.commands),
            output_func=print,
            max_steps=len(args.commands),
            enable_board_gui=False,
        )
    finally:
        main_module.run_command = original_run_command

    results = list(session.get("results", []))
    if len(results) != len(args.commands):
        raise RuntimeError(
            f"expected {len(args.commands)} results, collected {len(results)}"
        )

    rows: list[dict[str, object]] = []
    path_series: list[dict[str, object]] = []
    for index, (command_text, result) in enumerate(zip(args.commands, results), start=1):
        command = result["command"]
        actions = list(result.get("actions", []))
        execution = result["execution"]
        if not isinstance(execution, ExecutionResult):
            raise TypeError(f"result {index} did not include an ExecutionResult")

        step_label = f"step{index:02d}_{_slug_command(command_text)}"
        step_dir = output_dir / step_label
        step_dir.mkdir(parents=True, exist_ok=True)

        save_execution_csv(execution, output_dir=str(step_dir), label=step_label)
        save_plots(execution, output_dir=str(step_dir), label=step_label)

        desired_xyz = _fk_path(execution.desired_joint_angles, DEFAULT_CONFIG)
        actual_xyz = _fk_path(execution.actual_joint_angles, DEFAULT_CONFIG)
        path_length_m = _path_length(desired_xyz)
        actual_path_length_m = _path_length(actual_xyz)
        smoothness_proxy = _smoothness_proxy(execution.desired_joint_angles)
        safe_waypoint_ratio = _safe_waypoint_ratio(execution.obstacle_clearances)

        row = {
            "step": index,
            "command": command_text,
            "command_type": _command_type(command),
            "success": execution.success,
            "captures_piece": _captures_piece(actions),
            "primitive_count": result.get("primitive_count", 0),
            "trajectory_points": result.get("trajectory_points", 0),
            "execution_time_s": execution.execution_time,
            "wall_time_s": result.get("wall_time_s", timings[index - 1]),
            "path_length_m": round(path_length_m, 5),
            "actual_path_length_m": round(actual_path_length_m, 5),
            "smoothness_proxy_rad": round(smoothness_proxy, 7),
            "safe_waypoint_ratio": round(safe_waypoint_ratio, 4),
            "max_joint_error_rad": round(max(execution.joint_errors, default=0.0), 7),
            "mean_joint_error_rad": round(_mean(execution.joint_errors), 7),
            "max_ee_error_m": round(max(execution.end_effector_errors, default=0.0), 7),
            "mean_ee_error_m": round(_mean(execution.end_effector_errors), 7),
            "min_clearance_m": round(min(execution.obstacle_clearances, default=0.0), 5),
            "obstacle_ids": ";".join(result.get("obstacle_ids", [])),
            "safety_decisions": ";".join(
                decision.status for decision in result.get("safety_decisions", [])
            ),
        }
        row.update(summarize_execution(execution))
        rows.append(row)

        path_series.append(
            {
                "step": index,
                "command": command_text,
                "desired_xyz": desired_xyz,
                "actual_xyz": actual_xyz,
                "joint_error": execution.joint_errors,
                "ee_error": execution.end_effector_errors,
                "clearance": execution.obstacle_clearances,
            }
        )
        start_color = PALETTE[(index - 1) % len(PALETTE)]
        end_color = PALETTE[index % len(PALETTE)]
        _save_step_path_plot(
            desired_xyz,
            actual_xyz,
            command_text,
            step_dir / f"{step_label}_ee_path_xy.png",
            DEFAULT_CONFIG,
            start_color,
            end_color,
        )

    metrics_path = output_dir / "presentation_metrics.csv"
    _write_csv(metrics_path, rows)
    _save_dashboard(rows, output_dir / "presentation_metrics_dashboard.png")
    _save_error_ribbons(path_series, output_dir / "presentation_error_ribbons.png")
    _save_path_album(path_series, output_dir / "presentation_path_album.png", DEFAULT_CONFIG)
    _write_manifest(output_dir, args.commands, rows)

    _disconnect_pybullet()

    print(f"\nCollected {len(rows)} interactive commands")
    print(f"metrics: {metrics_path}")
    print(f"dashboard: {output_dir / 'presentation_metrics_dashboard.png'}")
    print(f"path album: {output_dir / 'presentation_path_album.png'}")


def _command_type(command: MoveCommand) -> str:
    if command.command_type == "move":
        return "move"
    return command.command_type


def _captures_piece(actions: list[LogicalAction]) -> bool:
    return any(action.cell.startswith("CAPTURED_") for action in actions)


def _fk_path(
    joint_path: list[tuple[float, ...]],
    config: Config,
) -> list[tuple[float, float, float]]:
    positions: list[tuple[float, float, float]] = []
    for joints in joint_path:
        if len(joints) < 6:
            continue
        x, y, z = solve_fk(tuple(joints[:6]), config)
        positions.append((float(x), float(y), float(z)))
    return positions


def _path_length(points: list[tuple[float, float, float]]) -> float:
    return sum(math.dist(a, b) for a, b in zip(points, points[1:]))


def _smoothness_proxy(joint_path: list[tuple[float, ...]]) -> float:
    if len(joint_path) < 3:
        return 0.0
    values: list[float] = []
    for a, b, c in zip(joint_path, joint_path[1:], joint_path[2:]):
        squared = [
            (c[j] - 2.0 * b[j] + a[j]) ** 2
            for j in range(min(len(a), len(b), len(c)))
        ]
        values.append(math.sqrt(sum(squared)))
    return _mean(values)


def _safe_waypoint_ratio(clearances: list[float]) -> float:
    if not clearances:
        return 0.0
    safe_like = sum(1 for value in clearances if value <= 0.02)
    return safe_like / len(clearances)


def _mean(values: list[float]) -> float:
    return sum(values) / len(values) if values else 0.0


def _write_csv(path: Path, rows: list[dict[str, object]]) -> None:
    if not rows:
        path.write_text("step\n", encoding="utf-8")
        return
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)


def _write_manifest(
    output_dir: Path,
    commands: list[str],
    rows: list[dict[str, object]],
) -> None:
    connection_info = None
    if p is not None and RUNTIME.client_id is not None and p.isConnected(RUNTIME.client_id):
        try:
            connection_info = p.getConnectionInfo(RUNTIME.client_id)
        except Exception:
            connection_info = None
    manifest = {
        "mode": "pybullet_direct_no_gui",
        "commands": commands,
        "result_count": len(rows),
        "pybullet_available": pybullet_available(),
        "client_id": RUNTIME.client_id,
        "connection_info": connection_info,
        "environment": {
            "CHESS_ROBOT_PYBULLET_GUI": os.environ.get("CHESS_ROBOT_PYBULLET_GUI"),
            "CHESS_ROBOT_BOARD_GUI": os.environ.get("CHESS_ROBOT_BOARD_GUI"),
        },
    }
    (output_dir / "run_manifest.json").write_text(
        json.dumps(manifest, indent=2, ensure_ascii=False),
        encoding="utf-8",
    )


def _slug_command(command: str) -> str:
    return (
        command.lower()
        .replace(" ", "_")
        .replace("-", "_")
        .replace("/", "_")
        .replace(":", "_")
    )


def _save_dashboard(rows: list[dict[str, object]], path: Path) -> None:
    labels = [f"{row['step']:02d}\n{row['command']}" for row in rows]
    x_values = list(range(len(rows)))
    colors = [PALETTE[i % len(PALETTE)] for i in x_values]

    fig, axes = plt.subplots(2, 2, figsize=(16, 9), constrained_layout=True)
    fig.patch.set_facecolor("#f8fafc")
    fig.suptitle(
        "Chinese Chess Robot - Headless Interactive Run Metrics",
        fontsize=18,
        fontweight="bold",
        color="#111827",
    )

    ax = axes[0][0]
    bars = ax.bar(
        x_values,
        [float(row["execution_time_s"]) for row in rows],
        color=colors,
        alpha=0.88,
    )
    ax.set_title("Execution Time per Command", loc="left", fontweight="bold")
    ax.set_ylabel("Simulated time (s)")
    ax.set_xticks(x_values, labels, fontsize=8)
    ax.grid(axis="y", alpha=0.25)
    _label_bars(ax, bars, "{:.1f}s")

    ax = axes[0][1]
    bars = ax.bar(
        x_values,
        [float(row["path_length_m"]) for row in rows],
        color=colors,
        alpha=0.85,
    )
    ax2 = ax.twinx()
    ax2.plot(
        x_values,
        [float(row["trajectory_points"]) for row in rows],
        color="#111827",
        linewidth=2.2,
        marker="o",
        markersize=5,
    )
    ax.set_title("End-Effector Path Length and Waypoint Count", loc="left", fontweight="bold")
    ax.set_ylabel("Path length (m)")
    ax2.set_ylabel("Waypoints")
    ax.set_xticks(x_values, labels, fontsize=8)
    ax.grid(axis="y", alpha=0.25)
    _label_bars(ax, bars, "{:.2f}m")

    ax = axes[1][0]
    joint_errors = [float(row["max_joint_error_rad"]) for row in rows]
    ee_errors_mm = [float(row["max_ee_error_m"]) * 1000.0 for row in rows]
    ax.plot(
        x_values,
        joint_errors,
        color="#b91c1c",
        marker="o",
        linewidth=2.4,
        label="Max joint error (rad)",
    )
    ax2 = ax.twinx()
    ax2.plot(
        x_values,
        ee_errors_mm,
        color="#1d4ed8",
        marker="s",
        linewidth=2.4,
        label="Max EE error (mm)",
    )
    ax.set_title("Tracking Error Peaks", loc="left", fontweight="bold")
    ax.set_ylabel("Joint error (rad)")
    ax2.set_ylabel("EE error (mm)")
    ax.set_xticks(x_values, labels, fontsize=8)
    ax.grid(axis="y", alpha=0.25)
    lines = ax.lines + ax2.lines
    ax.legend(lines, [line.get_label() for line in lines], loc="upper left", frameon=False)

    ax = axes[1][1]
    clearances_cm = [float(row["min_clearance_m"]) * 100.0 for row in rows]
    safe_ratio = [float(row["safe_waypoint_ratio"]) * 100.0 for row in rows]
    ax.bar(x_values, clearances_cm, color=colors, alpha=0.82, label="Min clearance (cm)")
    ax2 = ax.twinx()
    ax2.plot(
        x_values,
        safe_ratio,
        color="#111827",
        marker="D",
        linewidth=2.2,
        label="Safe-speed waypoint ratio",
    )
    ax.axhline(1.5, color="#991b1b", linestyle="--", linewidth=1.2, alpha=0.65)
    ax.set_title("Clearance and Safe-Speed Share", loc="left", fontweight="bold")
    ax.set_ylabel("Min clearance (cm)")
    ax2.set_ylabel("Safe ratio (%)")
    ax.set_xticks(x_values, labels, fontsize=8)
    ax.grid(axis="y", alpha=0.25)
    lines = [ax.patches[0]] if ax.patches else []
    ax2.legend(loc="upper right", frameon=False)

    for axis in axes.ravel():
        axis.set_facecolor("white")
        axis.spines["top"].set_visible(False)
        axis.spines["right"].set_visible(False)

    fig.savefig(path, dpi=220, bbox_inches="tight")
    plt.close(fig)


def _save_error_ribbons(series: list[dict[str, object]], path: Path) -> None:
    fig, axes = plt.subplots(2, 1, figsize=(15, 8), sharex=False, constrained_layout=True)
    fig.patch.set_facecolor("#f8fafc")
    fig.suptitle("Tracking Error Profiles", fontsize=17, fontweight="bold", color="#111827")

    for index, item in enumerate(series):
        color = PALETTE[index % len(PALETTE)]
        label = f"{item['step']:02d} {item['command']}"
        joint_error = list(item["joint_error"])
        ee_error_mm = [float(value) * 1000.0 for value in item["ee_error"]]
        axes[0].plot(
            _normalized_axis(joint_error),
            joint_error,
            color=color,
            linewidth=1.8,
            alpha=0.9,
            label=label,
        )
        axes[1].plot(
            _normalized_axis(ee_error_mm),
            ee_error_mm,
            color=color,
            linewidth=1.8,
            alpha=0.9,
            label=label,
        )

    axes[0].set_title("Joint Tracking Error", loc="left", fontweight="bold")
    axes[0].set_ylabel("RMS joint error (rad)")
    axes[1].set_title("End-Effector Error", loc="left", fontweight="bold")
    axes[1].set_ylabel("EE error (mm)")
    axes[1].set_xlabel("Normalized command progress")
    for axis in axes:
        axis.set_facecolor("white")
        axis.grid(alpha=0.25)
        axis.spines["top"].set_visible(False)
        axis.spines["right"].set_visible(False)
    axes[0].legend(ncol=4, fontsize=8, frameon=False, loc="upper right")

    fig.savefig(path, dpi=220, bbox_inches="tight")
    plt.close(fig)


def _save_path_album(
    series: list[dict[str, object]],
    path: Path,
    config: Config,
) -> None:
    rows = 2
    cols = 4
    fig, axes = plt.subplots(rows, cols, figsize=(16, 8), constrained_layout=True)
    fig.patch.set_facecolor("#f8fafc")
    fig.suptitle("Planned vs Actual End-Effector XY Paths", fontsize=17, fontweight="bold")
    axes_flat = list(axes.ravel())
    boundary_colors = _path_boundary_colors(len(series))

    for item_index, (axis, item) in enumerate(zip(axes_flat, series)):
        _draw_board(axis, config)
        _plot_path_pair(
            axis,
            item["desired_xyz"],
            item["actual_xyz"],
            boundary_colors[item_index],
            boundary_colors[item_index + 1],
        )
        axis.set_title(f"{item['step']:02d}. {item['command']}", loc="left", fontsize=10)

    for axis in axes_flat[len(series):]:
        _draw_path_legend_panel(axis)
    fig.savefig(path, dpi=220, bbox_inches="tight")
    plt.close(fig)


def _save_step_path_plot(
    desired_xyz: list[tuple[float, float, float]],
    actual_xyz: list[tuple[float, float, float]],
    command: str,
    path: Path,
    config: Config,
    start_color: str,
    end_color: str,
) -> None:
    fig, ax = plt.subplots(figsize=(8.5, 7.0))
    fig.patch.set_facecolor("#f8fafc")
    _draw_board(ax, config)
    _plot_path_pair(ax, desired_xyz, actual_xyz, start_color, end_color)
    ax.set_title(f"EE Path - {command}", loc="left", fontsize=14, fontweight="bold")
    ax.legend(handles=_path_legend_handles(), loc="upper right", frameon=False)
    fig.savefig(path, dpi=180, bbox_inches="tight")
    plt.close(fig)


def _draw_board(axis, config: Config) -> None:
    x0, y0, _ = config.board_origin
    width = (config.board_cols - 1) * config.cell_size
    height = (config.board_rows - 1) * config.cell_size
    axis.add_patch(
        Rectangle(
            (x0 - 0.03, y0 - 0.03),
            width + 0.06,
            height + 0.06,
            facecolor="#fff7ed",
            edgecolor="#c2410c",
            linewidth=1.2,
            alpha=0.45,
        )
    )
    for col in range(config.board_cols):
        x = x0 + col * config.cell_size
        axis.plot([x, x], [y0, y0 + height], color="#9a3412", linewidth=0.55, alpha=0.55)
    for row in range(config.board_rows):
        y = y0 + row * config.cell_size
        axis.plot([x0, x0 + width], [y, y], color="#9a3412", linewidth=0.55, alpha=0.55)
    river_y = y0 + 4.5 * config.cell_size
    axis.axhspan(
        river_y - 0.015,
        river_y + 0.015,
        color="#dbeafe",
        alpha=0.75,
        zorder=0,
    )
    for col in range(config.board_cols):
        for row in range(config.board_rows):
            axis.add_patch(
                Circle(
                    (x0 + col * config.cell_size, y0 + row * config.cell_size),
                    0.003,
                    facecolor="#7c2d12",
                    edgecolor="none",
                    alpha=0.45,
                )
            )
    axis.set_xlim(x0 - 0.06, x0 + width + 0.08)
    axis.set_ylim(y0 - 0.06, y0 + height + 0.06)
    axis.set_aspect("equal", adjustable="box")
    axis.set_xlabel("board x (m)")
    axis.set_ylabel("board y (m)")
    axis.grid(False)
    axis.set_facecolor("white")
    axis.spines["top"].set_visible(False)
    axis.spines["right"].set_visible(False)


def _plot_path_pair(
    axis,
    desired_xyz: list[tuple[float, float, float]],
    actual_xyz: list[tuple[float, float, float]],
    start_color: str,
    end_color: str,
) -> None:
    _expand_axis_to_paths(axis, desired_xyz, actual_xyz)
    if desired_xyz:
        _plot_gradient_xy_path(axis, desired_xyz, start_color, end_color)
        _add_direction_arrows(axis, desired_xyz, start_color, end_color)
        _mark_path_endpoint(axis, desired_xyz[0], start_color, "S", "o")
        _mark_path_endpoint(axis, desired_xyz[-1], end_color, "E", "s")
    if actual_xyz:
        axis.plot(
            [p[0] for p in actual_xyz],
            [p[1] for p in actual_xyz],
            color="#111827",
            linewidth=1.2,
            alpha=0.75,
            linestyle="--",
            zorder=2,
        )


def _expand_axis_to_paths(
    axis,
    *paths: list[tuple[float, float, float]],
) -> None:
    points = [point for path in paths for point in path]
    if not points:
        return
    xs = [point[0] for point in points]
    ys = [point[1] for point in points]
    x_min, x_max = axis.get_xlim()
    y_min, y_max = axis.get_ylim()
    margin = 0.045
    axis.set_xlim(min(x_min, min(xs) - margin), max(x_max, max(xs) + margin))
    axis.set_ylim(min(y_min, min(ys) - margin), max(y_max, max(ys) + margin))
    axis.set_aspect("equal", adjustable="box")


def _plot_gradient_xy_path(
    axis,
    points: list[tuple[float, float, float]],
    start_color: str,
    end_color: str,
) -> None:
    if len(points) < 2:
        return
    segments = [
        [(points[i][0], points[i][1]), (points[i + 1][0], points[i + 1][1])]
        for i in range(len(points) - 1)
    ]
    denom = max(len(segments) - 1, 1)
    colors = [
        _blend_color(start_color, end_color, i / denom)
        for i in range(len(segments))
    ]
    collection = LineCollection(
        segments,
        colors=colors,
        linewidths=2.7,
        alpha=0.96,
        capstyle="round",
        joinstyle="round",
        zorder=3,
    )
    axis.add_collection(collection)


def _add_direction_arrows(
    axis,
    points: list[tuple[float, float, float]],
    start_color: str,
    end_color: str,
) -> None:
    if len(points) < 4:
        return
    for fraction in (0.35, 0.7):
        index = min(max(int((len(points) - 1) * fraction), 0), len(points) - 2)
        start = points[index]
        end = points[index + 1]
        if math.hypot(end[0] - start[0], end[1] - start[1]) < 1e-5:
            continue
        arrow = FancyArrowPatch(
            (start[0], start[1]),
            (end[0], end[1]),
            arrowstyle="-|>",
            mutation_scale=9,
            linewidth=1.0,
            color=_blend_color(start_color, end_color, fraction),
            alpha=0.92,
            zorder=5,
        )
        axis.add_patch(arrow)


def _mark_path_endpoint(
    axis,
    point: tuple[float, float, float],
    color: str,
    label: str,
    marker: str,
) -> None:
    axis.scatter(
        [point[0]],
        [point[1]],
        s=94,
        marker=marker,
        facecolor="white",
        edgecolor=color,
        linewidth=2.6,
        zorder=6,
        clip_on=True,
    )
    axis.text(
        point[0],
        point[1],
        label,
        ha="center",
        va="center",
        fontsize=7,
        fontweight="bold",
        color=color,
        zorder=7,
        clip_on=True,
    )


def _path_boundary_colors(step_count: int) -> list[str]:
    return [PALETTE[index % len(PALETTE)] for index in range(step_count + 1)]


def _blend_color(start_color: str, end_color: str, fraction: float) -> tuple[float, float, float, float]:
    import matplotlib.colors as mcolors

    fraction = min(1.0, max(0.0, fraction))
    start = mcolors.to_rgba(start_color)
    end = mcolors.to_rgba(end_color)
    return tuple(
        start[i] + (end[i] - start[i]) * fraction
        for i in range(4)
    )


def _path_legend_handles() -> list[Line2D]:
    return [
        Line2D([0], [0], color="#2563eb", linewidth=2.7, label="planned: start to end gradient"),
        Line2D([0], [0], color="#111827", linewidth=1.2, linestyle="--", label="actual"),
        Line2D([0], [0], marker="o", color="none", markerfacecolor="white",
               markeredgecolor="#2563eb", markeredgewidth=2.2, markersize=8, label="S start"),
        Line2D([0], [0], marker="s", color="none", markerfacecolor="white",
               markeredgecolor="#dc2626", markeredgewidth=2.2, markersize=8, label="E end"),
    ]


def _draw_path_legend_panel(axis) -> None:
    axis.axis("off")
    axis.set_title("Legend", loc="left", fontsize=11, fontweight="bold")
    legend = axis.legend(
        handles=_path_legend_handles(),
        loc="upper left",
        frameon=False,
        fontsize=9,
        handlelength=2.4,
        borderaxespad=0.0,
    )
    axis.add_artist(legend)
    axis.text(
        0.0,
        0.36,
        "Boundary colors are shared:\n"
        "step i end color = step i+1 start color.\n"
        "The gradient shows command progress.",
        transform=axis.transAxes,
        ha="left",
        va="top",
        fontsize=9,
        color="#374151",
        linespacing=1.45,
    )


def _normalized_axis(values: list[float]) -> list[float]:
    if len(values) <= 1:
        return [0.0 for _ in values]
    return [i / (len(values) - 1) for i in range(len(values))]


def _label_bars(axis, bars, template: str) -> None:
    for bar in bars:
        height = bar.get_height()
        axis.text(
            bar.get_x() + bar.get_width() / 2.0,
            height,
            template.format(height),
            ha="center",
            va="bottom",
            fontsize=8,
            color="#374151",
        )


def _disconnect_pybullet() -> None:
    if p is None or RUNTIME.client_id is None:
        return
    try:
        if p.isConnected(RUNTIME.client_id):
            p.disconnect(RUNTIME.client_id)
    except Exception:
        pass


if __name__ == "__main__":
    main()
