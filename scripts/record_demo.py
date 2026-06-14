#!/usr/bin/env python3
"""批量 Demo 录制脚本。

运行预定义的多个场景并自动保存：
- 每个场景的 error/clearance 曲线图 (PNG)
- 每个场景的逐点执行数据 (CSV)
- 场景对比汇总表 (CSV)

用法:
    python scripts/record_demo.py              # 运行全部场景
    python scripts/record_demo.py --quick      # 快速模式（少场景）
    python scripts/record_demo.py --scenario move  # 只跑指定场景
"""

from __future__ import annotations

import argparse
import os
import sys
import time
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT))

from src.common.config import DEFAULT_CONFIG
from src.common.types import MoveCommand
from src.control.controller import execute_trajectory
from src.control.logger import save_summary_csv, summarize_execution
from src.interaction.board_state import create_initial_board, make_logical_actions
from src.interaction.chess_rules import validate_move
from src.interaction.cli import parse_command
from src.planning.motion_primitives import build_motion_primitives, get_action_primitive_ranges
from src.planning.obstacle_map import build_primitive_obstacle_contexts
from src.planning.trajectory_planner import plan_trajectory
from src.simulation.attachment import attach_piece, detach_piece
from src.simulation.load_robot import load_robot
from src.simulation.scene_builder import build_scene, set_human_safety_zone
from src.visualization.plot_results import save_execution_csv, save_plots


# ── 预定义 Demo 场景 ──

SCENARIOS: dict[str, dict] = {
    # ── 基线场景 ──
    "simple_move": {
        "label": "01_simple_move",
        "description": "简单走子 A1→A2（无障碍基线）",
        "commands": ["A1 A2"],
        "obstacle_mode": "none",
        "notes": "无预设柱。Baseline: 插值+smooth，直线最短路径。",
    },
    "long_horizontal": {
        "label": "02_long_horizontal",
        "description": "炮长距离横移 B3→G3 (row=3)",
        "commands": ["B3 G3"],
        "obstacle_mode": "none",
        "notes": "0.30m 水平移动 (col B→G)。C3-G3 为空，合法。"
                "验证 waypoint 插值均匀、关节平滑有效。",
    },
    "long_vertical": {
        "label": "03_long_vertical",
        "description": "车长距离纵移 A1→A9",
        "commands": ["A1 A9"],
        "obstacle_mode": "none",
        "notes": "0.48m 垂直移动穿越棋盘。transfer 从 row=0→row=8。",
    },
    # ── 障碍物场景 ──
    "obstacle_gate": {
        "label": "04_obstacle_gate",
        "description": "双柱门洞下走子 (mode_2, C5+F5)",
        "commands": ["obstacle_mode 2", "B3 G3"],
        "obstacle_mode": "mode_2",
        "notes": "B3→G3 row=3 路径低于障碍柱 (row=5)，不触发绕行。"
                "验证 obstacle_mode 切换和场景重建。",
    },
    "obstacle_wall": {
        "label": "05_obstacle_wall",
        "description": "三柱墙下走子 (mode_3, C5+E6+G5)",
        "commands": ["obstacle_mode 3", "A1 A9"],
        "obstacle_mode": "mode_3",
        "notes": "3 个预设柱更密集。A1→A9 transfer 沿 x=0 通过，距离 C5=0.12m。",
    },
    # ── 吃子场景 ──
    "capture": {
        "label": "06_capture",
        "description": "吃子 — 两步设局: 车移近→吃黑车",
        "commands": ["A1 A9", "A9 A10"],
        "obstacle_mode": "none",
        "notes": "Step1: 红车 A1→A9 (接近黑方底线)。"
                "Step2: A9→A10 吃掉黑车。"
                "验证 pick→place attach/detach 切换、captured area 正确放置。",
    },
    # ── 人手安全区场景 ──
    "hand_detour": {
        "label": "07_hand_detour",
        "description": "人手安全区 — 车纵移横穿手区上方",
        "commands": ["hand_on", "A1 A8", "hand_off"],
        "obstacle_mode": "none",
        "notes": "手区: 中心(0.24,0.42,z_safe), X轴长0.24m (col≈4-5), Y宽0.05m (row≈7)。"
                "验证 hand_on/off 切换、obstacle_map 中 HORIZONTAL_CYLINDER 构造。",
    },
    # ── 完整流程 ──
    "full_workflow": {
        "label": "08_full_workflow",
        "description": "完整流程: 走子×2 → reset",
        "commands": ["A1 A2", "A2 A3", "reset"],
        "obstacle_mode": "none",
        "notes": "三步演示: 车上移两次(A1→A2→A3)→reset 恢复初始。连续走子+状态跟踪+复位。",
    },
    # ── A* 绕行场景 ──
    "obstacle_detour": {
        "label": "09_obstacle_detour",
        "description": "A*绕行 — 车横移穿障碍柱 (mode_3, A5→I5)",
        "commands": ["obstacle_mode 3", "A1 A5", "A5 I5"],
        "obstacle_mode": "mode_3",
        "notes": "Step1: 车 A1→A5 纵移(row=0→4)，把车送到 row=5 高度。"
                "Step2: 车 A5→I5 横移，直接路径穿过 C5+G5 障碍柱 (半径0.045+margin0.015=0.06m)。"
                "碰撞检测在 z=z_safe 平面阻止直线通行 → A* 产生绕行路径。"
                "GUI 中可见：绿色直线路径 (被阻挡)，红色 A* 绕行路径。",
    },
}


# ── 快速模式子集 ──
QUICK_SCENARIOS = ["simple_move", "long_horizontal", "obstacle_wall", "hand_detour"]


def run_scenario(
    scenario_name: str,
    scenario: dict,
    output_dir: Path,
    config=DEFAULT_CONFIG,
) -> dict[str, object]:
    """运行一个 demo 场景并保存所有结果。

    Returns:
        场景 summary dict（用于汇总表）
    """
    label = scenario["label"]
    print(f"\n{'='*60}")
    print(f"[{label}] {scenario['description']}")
    print(f"   备注: {scenario['notes']}")
    print(f"{'='*60}")

    robot = load_robot()
    scene = build_scene(config=config, obstacle_mode=scenario["obstacle_mode"])
    board = create_initial_board()
    human_hand_present = False

    all_summaries: list[dict] = []
    scenario_dir = output_dir / label
    scenario_dir.mkdir(parents=True, exist_ok=True)

    for cmd_idx, command_text in enumerate(scenario["commands"]):
        command = parse_command(command_text)

        # Handle meta-commands
        if command.command_type == "hand_on":
            human_hand_present = True
            set_human_safety_zone(True, config)
            print(f"  [cmd {cmd_idx}] hand_on: 人手区已激活")
            continue
        elif command.command_type == "hand_off":
            human_hand_present = False
            set_human_safety_zone(False, config)
            print(f"  [cmd {cmd_idx}] hand_off: 人手区已关闭")
            continue
        elif command.command_type == "obstacle_mode":
            scene = build_scene(config=config, obstacle_mode=command.mode)
            print(f"  [cmd {cmd_idx}] obstacle_mode → {command.mode}")
            continue

        # Regular move command
        validation = validate_move(board, command)
        if not validation.is_legal:
            print(f"  [cmd {cmd_idx}] SKIP: {validation.reason}")
            continue

        t0 = time.perf_counter()
        actions = make_logical_actions(board, command)
        primitives = build_motion_primitives(actions, config)
        contexts = build_primitive_obstacle_contexts(
            actions=actions, primitives=primitives, board=board,
            extra_obstacles=scene.obstacles,
            human_hand_present=human_hand_present, config=config,
        )
        trajectory = plan_trajectory(contexts, config=config)

        # Split execution by action for correct attach/detach
        action_ranges = get_action_primitive_ranges(actions)
        accumulated = []
        attached_id = ""
        for i, action in enumerate(actions):
            start, end = action_ranges[i]
            from src.common.types import JointTrajectory
            segment = JointTrajectory(
                joint_waypoints=trajectory.joint_waypoints[start:end],
                speed_profile=trajectory.speed_profile[start:end],
            )
            if action.action_type == "pick" and action.piece_id:
                attached_id = action.piece_id
                attach_piece(piece_id=action.piece_id, end_effector_id=robot.end_effector_id)
            seg_result = execute_trajectory(segment, config=config)
            accumulated.append(seg_result)
            if action.action_type == "place" and attached_id:
                detach_piece(piece_id=attached_id)
                attached_id = ""

        from main import _merge_executions, apply_logical_actions
        execution = _merge_executions(accumulated)
        if execution.success:
            apply_logical_actions(board, actions)

        elapsed = time.perf_counter() - t0
        summary = summarize_execution(execution)
        summary.update({
            "scenario": label,
            "command": command_text,
            "primitive_count": len(primitives),
            "trajectory_points": len(trajectory.joint_waypoints),
            "human_hand_present": human_hand_present,
            "obstacle_mode": scenario["obstacle_mode"],
            "planning_time_s": round(elapsed, 4),
            "search_detours": _count_detours(contexts),
        })
        all_summaries.append(summary)

        # Save per-command plots and CSV
        cmd_label = f"{label}_cmd{cmd_idx}"
        saved_plots = save_plots(execution, output_dir=str(scenario_dir), label=cmd_label)
        saved_csv = save_execution_csv(execution, output_dir=str(scenario_dir), label=cmd_label)
        print(f"  [cmd {cmd_idx}] {command_text}: {len(primitives)} primitives, "
              f"{len(trajectory.joint_waypoints)} waypoints, "
              f"{elapsed:.3f}s, "
              f"max_joint_err={summary['max_joint_error']:.4f}, "
              f"max_ee_err={summary['max_end_effector_error']:.4f}")
        print(f"           plots: {len(saved_plots)} files → {scenario_dir}")
        print(f"           csv:   {saved_csv}")

    return {
        "scenario": label,
        "description": scenario["description"],
        "summaries": all_summaries,
    }


def main():
    parser = argparse.ArgumentParser(description="批量 Demo 录制")
    parser.add_argument("--quick", action="store_true", help="快速模式（少场景）")
    parser.add_argument("--scenario", type=str, help="只跑指定场景名")
    parser.add_argument("--output", type=str, default="results", help="输出目录")
    args = parser.parse_args()

    output_dir = Path(args.output)
    output_dir.mkdir(parents=True, exist_ok=True)

    if args.scenario:
        names = [args.scenario]
    elif args.quick:
        names = QUICK_SCENARIOS
    else:
        names = list(SCENARIOS.keys())

    all_scenario_summaries: list[dict] = []

    for name in names:
        if name not in SCENARIOS:
            print(f"Unknown scenario: {name}")
            continue
        result = run_scenario(name, SCENARIOS[name], output_dir)
        all_scenario_summaries.append(result)

    # ── 生成汇总对比表 ──
    summary_rows: list[dict] = []
    for scenario_result in all_scenario_summaries:
        for s in scenario_result["summaries"]:
            summary_rows.append(s)

    if summary_rows:
        summary_path = save_summary_csv(summary_rows, str(output_dir / "summary_table.csv"))
        print(f"\n{'='*60}")
        print(f"汇总表: {summary_path}")
        print(f"共 {len(summary_rows)} 条命令, {len(all_scenario_summaries)} 个场景")
        print(f"输出目录: {output_dir.resolve()}")
        print(f"{'='*60}")

        # Print summary to console
        print(f"\n{'场景':<30} {'命令':<12} {'max_joint_err':>12} {'max_ee_err':>12} {'min_clear':>10} {'time':>8}")
        print("-" * 90)
        for row in summary_rows:
            print(f"{row.get('scenario',''):<30} {row.get('command',''):<12} "
                  f"{row.get('max_joint_error',0):>12.6f} {row.get('max_end_effector_error',0):>12.6f} "
                  f"{row.get('min_obstacle_clearance',0):>10.4f} {row.get('execution_time',0):>8.3f}")


def _count_detours(contexts) -> int:
    """统计有多少个 primitive 触发了 A* 绕行。"""
    # 间接指标：speed_profile 中 "safe" 的比例（绕行全程=safe）
    # 直接指标需要从 plan_trajectory 返回，这里用 heuristic
    return sum(1 for ctx in contexts if ctx.safety_decision.status != "continue")


if __name__ == "__main__":
    main()
