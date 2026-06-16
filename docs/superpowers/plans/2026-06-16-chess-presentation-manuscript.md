# Chess Presentation Manuscript Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use $superpower-subagents (recommended) or $superpower-executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking via update_plan.

**Goal:** Write a polished Chinese Markdown presentation manuscript for the UR5 Chinese chess robot project, including project framing, engineering process, technical highlights, challenges, validation, and a plan for additional visualization data.

**Architecture:** This is a documentation deliverable, not a code feature. The manuscript will synthesize existing docs, source modules, tests, and results into one narrative file under `presentation/`, while clearly separating current evidence from future visualization-data plans.

**Tech Stack:** Markdown, existing Python/PyBullet/URDF project evidence, existing docs under `docs/`, source code under `src/`, tests under `tests/`, and result artifacts under `results/`.

---

### Task 1: Prepare Evidence Map

**Files:**
- Read: `docs/项目大纲project_outline.md`
- Read: `docs/统一接口文档.md`
- Read: `docs/path_planning_guide.md`
- Read: `docs/superpowers/specs/2026-06-16-chess-presentation-manuscript-design.md`
- Read: `src/common/types.py`
- Read: `src/planning/trajectory_planner.py`
- Read: `scripts/record_demo.py`
- Read: `results/summary_table.csv`

- [ ] **Step 1: Verify current worktree state**

Run:

```powershell
git status --short
```

Expected: only unrelated existing changes may remain; do not modify or stage `docs/项目大纲project_outline.md`.

- [ ] **Step 2: Re-read the approved spec**

Run:

```powershell
[Console]::OutputEncoding=[System.Text.Encoding]::UTF8; Get-Content -Encoding UTF8 -LiteralPath 'docs\superpowers\specs\2026-06-16-chess-presentation-manuscript-design.md'
```

Expected: spec includes the visualization-data expansion requirement and writing boundaries.

- [ ] **Step 3: Collect source-backed points**

Use the gathered files to extract these evidence points:

```text
Project scope: simulation validation, not chess AI or real hardware.
Architecture: A/B/C/D modules and shared types.
Pipeline: MoveCommand -> LogicalAction -> MotionPrimitive -> JointTrajectory -> ExecutionResult.
Planning: collision checking, occupancy grid, A* search, interpolation, smoothing, Fast/Safe mode.
Validation: FK/EE error, joint error, obstacle clearance, CSV/PNG outputs.
Future data plan: path length, planning time, smoothness, speed-mode ratio, clearance distribution, trajectory comparison.
```

### Task 2: Draft The Manuscript

**Files:**
- Create: `presentation/Chess_Robot_Project_Showcase.md`

- [ ] **Step 1: Create the document skeleton**

Create `presentation/Chess_Robot_Project_Showcase.md` with these top-level sections:

```markdown
# 交互式中国象棋机械臂仿真系统项目展示稿

## 1. 项目一句话概括
## 2. 立项思考：我们为什么这样定义问题
## 3. 系统架构：四层协作与统一接口
## 4. 从一步棋到机械臂动作
## 5. 技术栈与核心亮点
## 6. 避障、安全与轨迹质量
## 7. 控制执行与结果验证
## 8. 我们遇到的关键困难，以及如何解决
## 9. 展示增强数据计划
## 10. 建议展示顺序
## 11. 总结：这个项目真正展示了什么
```

- [ ] **Step 2: Write current-system sections**

Fill sections 1-7 using project evidence. Keep the tone suitable for a presentation speaker note: clear, confident, and technically grounded.

- [ ] **Step 3: Write challenge-and-solution sections**

Fill section 8 with problem-analysis-solution-result units covering:

```text
Scope control and project boundaries.
Four-person parallel development and interface contracts.
Persistent board/session state.
Chinese notation and simplified chess rules.
UR5 pose/IK consistency.
Obstacle modelling and path planning.
Trajectory smoothness and verification outputs.
```

- [ ] **Step 4: Write visualization-data expansion plan**

In section 9, describe planned visualization data and a proposed module:

```text
Suggested module: scripts/collect_presentation_metrics.py
Inputs: predefined scenario list, current config, optional obstacle mode, optional human hand state.
Outputs: CSV summary, per-scenario PNG charts, comparison tables.
Metrics: path length, planning time, trajectory point count, smoothness proxy, speed profile ratio, min clearance, max joint error, max EE error.
Status wording: planned / recommended / can be added next, not already completed.
```

### Task 3: Boundary And Quality Verification

**Files:**
- Verify: `presentation/Chess_Robot_Project_Showcase.md`

- [ ] **Step 1: Search for forbidden or risky wording**

Run:

```powershell
[Console]::OutputEncoding=[System.Text.Encoding]::UTF8; Select-String -LiteralPath 'presentation\Chess_Robot_Project_Showcase.md' -Pattern 'AI|ai|固定避障|假避障|固定.*轨迹|fake|fixed|TODO|TBD|待定'
```

Expected: no real matches except harmless substrings inside ordinary file names if any; revise if matches imply forbidden content or placeholders.

- [ ] **Step 2: Confirm required sections exist**

Run:

```powershell
[Console]::OutputEncoding=[System.Text.Encoding]::UTF8; Select-String -LiteralPath 'presentation\Chess_Robot_Project_Showcase.md' -Pattern '^## '
```

Expected: all planned sections appear, including `展示增强数据计划`.

- [ ] **Step 3: Verify current evidence and future plans are separated**

Read section 9 manually and ensure future data/module content uses planned wording, not completed-result wording.

- [ ] **Step 4: Check git status**

Run:

```powershell
git status --short
```

Expected: new/modified files from this task only, plus the pre-existing `docs/项目大纲project_outline.md` user change.

### Task 4: Final Report

**Files:**
- Report: `presentation/Chess_Robot_Project_Showcase.md`

- [ ] **Step 1: Summarize deliverable**

Report the created manuscript path and mention that the planned visualization data module is included as a future enhancement section.

- [ ] **Step 2: Mention verification**

Report the boundary scan and section check results. If tests are not run because the work is documentation-only, say so plainly.
