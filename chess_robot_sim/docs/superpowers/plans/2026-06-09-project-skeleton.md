# Chess Robot Simulation Project Skeleton Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use $superpower-subagents (recommended) or $superpower-executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking via update_plan.

**Goal:** Build the 6/9 technical lead handoff: a runnable project skeleton, stable module interfaces, and per-member task files.

**Architecture:** The project is split by responsibility: `interaction` for commands and board logic, `simulation` for PyBullet-facing scene hooks, `planning` for coordinates/IK/trajectory, `control` for execution/logging, and `common` for shared types/config. `main.py` wires modules through mock-safe interfaces so every teammate can start without waiting for full PyBullet behavior.

**Tech Stack:** Python 3, standard-library dataclasses/enums/unittest for the skeleton, optional PyBullet dependency for later simulation implementation.

---

### Task 1: Interface Contract Tests

**Files:**
- Create: `chess_robot_sim/tests/test_contract_interfaces.py`

- [ ] Write tests that import the desired module interfaces and run a mock command through interaction, planning, simulation, and control modules.
- [ ] Run `python -m unittest discover -s chess_robot_sim/tests -v` and confirm it fails because the skeleton modules do not exist yet.

### Task 2: Common Types And Config

**Files:**
- Create: `chess_robot_sim/src/common/types.py`
- Create: `chess_robot_sim/src/common/config.py`
- Create: `chess_robot_sim/src/common/__init__.py`

- [ ] Define dataclasses/enums for cells, pieces, commands, logical actions, motion primitives, obstacles, trajectories, and execution results.
- [ ] Define a single `Config` object for board geometry, obstacle inflation, home pose, and Fast/Safe speed scales.

### Task 3: Module Stubs

**Files:**
- Create: files under `src/interaction`, `src/simulation`, `src/planning`, `src/control`, and `src/visualization`.

- [ ] Add focused functions with docstrings and mock-safe return values.
- [ ] Keep `main.py`, `src/common`, and interface documentation under C ownership.

### Task 4: Documentation And Member TODOs

**Files:**
- Create: `chess_robot_sim/docs/interface_contract.md`
- Create: `chess_robot_sim/docs/member_todos/A_interaction_todo.md`
- Create: `chess_robot_sim/docs/member_todos/B_simulation_todo.md`
- Create: `chess_robot_sim/docs/member_todos/C_technical_lead_todo.md`
- Create: `chess_robot_sim/docs/member_todos/D_control_validation_todo.md`
- Create: `chess_robot_sim/README.md`

- [ ] Document every module owner, file, public function, input, output, and mock behavior.
- [ ] Give each teammate exact starting files and acceptance criteria.

### Task 5: Verification

**Files:**
- Modify: `chess_robot_sim/main.py`

- [ ] Run `python -m unittest discover -s chess_robot_sim/tests -v`.
- [ ] Run `python chess_robot_sim/main.py --demo`.
- [ ] Run `python -m compileall chess_robot_sim`.
