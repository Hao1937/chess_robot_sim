# Demo 录制 & PyBullet 集成 — 交接文档

**日期：** 2026-06-14
**状态：** D 模块逻辑部分已完成，PyBullet 物理驱动待集成
**前置阅读：** `docs/path_planning_guide.md`、`docs/统一接口文档.md`

---

## 1. 本次已完成

### 1.1 D 模块新增/修改文件

| 文件 | 变更 | 关键点 |
|------|------|--------|
| `src/control/fk_solver.py` | **新增** | UR5 FK（DH 参数与 IK 一致），`solve_fk(joint_angles) → (x,y,z)`，精度 < 0.03mm |
| `src/control/controller.py` | **升级** | EE 误差改用 FK 计算（原为 joint_error × 1.6）；加关节限位 clamp；brownian drift 噪声 |
| `scripts/record_demo.py` | **新增** | 8 场景批量录制：PNG 曲线 + CSV 数据 + 汇总表 |
| `tests/test_contract_interfaces.py` | **修复** | 1 处：trajectory waypoint 从 3 关节 → 6 关节 |

### 1.2 录制脚本用法

```bash
python scripts/record_demo.py              # 全部 8 场景 (~30s)
python scripts/record_demo.py --quick      # 快速 4 场景 (~15s)
python scripts/record_demo.py --scenario capture  # 单场景

# 输出目录: results/
#   每个场景一个子目录，含 3 张 PNG + 1 个 CSV
#   summary_table.csv = 全部场景对比汇总
```

### 1.3 路径规划模块（关联阅读）

`docs/path_planning_guide.md` — 本次同步完成的 A* + 碰撞检测 + 轨迹平滑完整技术文档。

---

## 2. PyBullet 集成 —— 待完成

### 2.1 当前状态

```
PyBullet            ❌ 未安装 (pip install pybullet)
GUI 模式            ❌ 未启用 (需 CHESS_ROBOT_PYBULLET_GUI=1)
机械臂物理驱动       ❌ execute_trajectory() 只产出数据，不控制 PyBullet 关节
路径可视化           ❌ A* 绕行路径未在 GUI 中画线
人手区 3D 可视化     ❌ 水平圆柱 (B 有代码但未在 GUI 验证)
```

### 2.2 第一步：安装 PyBullet

```bash
pip install pybullet
```

验证：
```bash
python -c "import pybullet; print('OK')"
```

### 2.3 第二步：D 的 controller 接入 PyBullet 关节控制

**文件：** `src/control/controller.py`
**函数：** `execute_trajectory()`

当前是纯数据 mock（加噪声生成 `ExecutionResult`）。需要改为真正驱动 PyBullet 机械臂。

**核心改动：**

```python
# 当前 (mock):
actual_waypoint = tuple(value + noise for value in waypoint)

# 目标 (PyBullet):
p.setJointMotorControlArray(
    bodyUniqueId=robot_id,
    jointIndices=joint_indices,
    controlMode=p.POSITION_CONTROL,
    targetPositions=waypoint,
    targetVelocities=[0]*6,
    forces=[100]*6,   # 或根据 speed_mode 调整
    physicsClientId=client_id,
)
# Step simulation
p.stepSimulation(client_id)
# Read actual joint states
actual_waypoint = tuple(
    p.getJointState(robot_id, j, client_id)[0] for j in joint_indices
)
```

**注意事项：**
- 需要在 `execute_trajectory()` 签名中增加 `robot_id`、`joint_indices`、`client_id` 参数，**或通过全局 `RUNTIME` 单例获取**
- 建议通过 `src/simulation/_runtime.py` 的 `RUNTIME` 拿 `client_id`、`robot_id`、`joint_indices`
- 速度模式：fast → `forces` 大 / `maxVelocity` 高；safe → 慢
- `step_time` 应改为实际仿真步长时间，不再用固定 0.15s/0.45s
- 保持返回值 `ExecutionResult` 不变（接口兼容）
- 需要 `from src.simulation._runtime import RUNTIME` 或改为参数注入

### 2.4 第三步：路径可视化

在 PyBullet GUI 中画线展示规划路径 vs 直接路径。

**建议实现：**

```python
# src/planning/visualization.py (新文件) 或放在 trajectory_planner 中

def draw_path_debug(path_xyz, color, client_id):
    """在 PyBullet 中用 p.addUserDebugLine 画路径。"""
    for i in range(len(path_xyz) - 1):
        p.addUserDebugLine(
            path_xyz[i], path_xyz[i+1],
            lineColorRGB=color,
            lineWidth=2.0,
            lifeTime=5.0,  # 5 秒后消失
            physicsClientId=client_id,
        )
```

**调用时机：** `plan_trajectory()` 内，当 A* 绕行被触发时，将绕行路径点用红色画出，直接路径用绿色虚线画出。

### 2.5 第四步：GUI 模式全流程测试

```bash
set CHESS_ROBOT_PYBULLET_GUI=1
python main.py --demo --command "A1 A2"
```

预期：PyBullet GUI 窗口打开 → 棋盘 + 棋子 + 机械臂显示 → 机械臂运动完成走子。

### 2.6 第五步：录制 Demo

```bash
set CHESS_ROBOT_PYBULLET_GUI=1
python scripts/record_demo.py
# 同时用 OBS / Xbox Game Bar 录屏
```

或用 PyBullet 自带的录制：
```python
import pybullet as p
p.startStateLogging(p.STATE_LOGGING_VIDEO_MP4, "demo.mp4", physicsClientId=client_id)
```

---

## 3. 需要关注的接口

| 接口 | 由谁提供 | D 怎么用 |
|------|----------|----------|
| `RUNTIME.client_id` | B (`_runtime.py`) | `p.setJointMotorControlArray(client_id=...)` |
| `RUNTIME.robot_id` | B (`load_robot.py`) | 同上 `bodyUniqueId=...` |
| `RUNTIME.joint_indices` | B | `jointIndices=...` |
| `RUNTIME.end_effector_id` | B | 末端 link index，FK 验证用 |
| `JointTrajectory` | C (`trajectory_planner.py`) | 输入，waypoints 已含插值+平滑 |
| `ExecutionResult` | D | 输出，**接口不能变** |

**原则：** D 的 `execute_trajectory()` 签名可以加参数（如 `robot_id`），但 `ExecutionResult` 的字段不能变 —— A/B/C/main.py 都依赖它。

---

## 4. 测试

```bash
# 全部测试（无需 PyBullet）
python -m unittest discover -s tests -v   # 51 tests

# 单独测 D 模块
python -m unittest tests.test_path_planning -v  # 29 tests
```

新增 PyBullet 相关测试应放在 `tests/test_contract_interfaces.py` 的 `test_simulation_and_control_stubs_share_contract` 附近，且需 `@unittest.skipIf(p is None, "PyBullet not installed")`。

---

## 5. Demo 录制场景清单

`scripts/record_demo.py` 中预定义了 8 个场景：

| # | 场景 | 走法 | 演示要点 |
|---|------|------|----------|
| 1 | simple_move | A1→A2 | 基线: 无障碍插值+smooth |
| 2 | long_horizontal | B3→G3 | 0.30m 水平插值均匀性 |
| 3 | long_vertical | A1→A9 | 0.48m 垂直移动 |
| 4 | obstacle_gate | mode_2, B3→G3 | 障碍柱切换+重建 |
| 5 | obstacle_wall | mode_3, A1→A9 | 三柱密集布局 |
| 6 | capture | A1→A9→A10 | 两步设局吃黑车，attach/detach |
| 7 | hand_detour | hand_on, A1→A8, hand_off | HORIZONTAL_CYLINDER 精确建模 |
| 8 | full_workflow | A1→A2→A3→reset | 连续走子+状态跟踪+复位 |

**注意：** 当前场景的走法路径未穿过障碍柱核心区（因合法象棋走法限制）。需要设计专门穿过障碍柱的 A* 绕行场景，或通过自定义初始棋盘布局来实现。

---

## 6. 待办优先级

```
P0 (必须):
  [ ] pip install pybullet
  [ ] execute_trajectory() 接入 PyBullet 关节控制
  [ ] GUI 模式全流程验证

P1 (提升 demo 质量):
  [ ] 路径可视化 (A* 绕行 vs 直接路径 debug lines)
  [ ] 人手区 3D 可视化验证
  [ ] 设计穿过障碍区的绕行 demo 场景

P2 (锦上添花):
  [ ] PyBullet 录像 (startStateLogging)
  [ ] 实时速度优化
  [ ] 音效/解说
```
