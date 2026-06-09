# 成员 D TODO：控制执行、日志、曲线、验证结果

你负责 `src/control/`、`src/visualization/` 和 `results/`。

你的模块回答一个问题：**机器人有没有按轨迹动准，误差和避障效果怎么证明？**

不要写棋盘规则、场景搭建、IK、轨迹规划。你接收 C 的 `JointTrajectory`，输出 `ExecutionResult` 和图表数据。

## 你负责的文件

| 文件 | 你要做什么 |
|---|---|
| `src/control/controller.py` | 执行 joint trajectory |
| `src/control/logger.py` | 记录和汇总执行数据 |
| `src/visualization/plot_results.py` | 生成曲线数据或图片 |
| `results/` | 保存 csv、png、结果表 |

## 必须遵守的接口

从 `src/common/types.py` 使用 `JointTrajectory` 和 `ExecutionResult`。

保持这些函数名不变：

```python
def execute_trajectory(trajectory: JointTrajectory) -> ExecutionResult:

def summarize_execution(execution: ExecutionResult) -> dict[str, float | bool]:

def build_plot_data(execution: ExecutionResult) -> dict[str, list[float]]:
```

## 6/11 周四：控制器和日志起步

### 1. 控制器接口

文件：`src/control/controller.py`

函数：

```python
def execute_trajectory(trajectory: JointTrajectory) -> ExecutionResult:
```

要求：能接收 C 输出的 `JointTrajectory`；先保留 mock 也可以，但必须返回完整 `ExecutionResult`；后续接 PyBullet 时，在这里实现 joint-space position control 或 PID。

`ExecutionResult` 至少包含：`success`、`desired_joint_angles`、`actual_joint_angles`、`joint_errors`、`end_effector_errors`、`obstacle_clearances`、`execution_time`。

### 2. 日志汇总

文件：`src/control/logger.py`

函数：

```python
def summarize_execution(execution: ExecutionResult) -> dict[str, float | bool]:
```

必须输出：`success`、`max_joint_error`、`max_end_effector_error`、`min_obstacle_clearance`、`execution_time`。

## 6/14 周日：P0 验证

目标：给普通走子和吃子初版输出误差数据。

你要做：`execute_trajectory()` 能被 `main.py --demo` 调用；能记录 desired vs actual joint angle；能输出 end-effector position error；能把结果 summary 打印或保存。

如果 B/C 还没接真实 PyBullet：继续用 mock 数据，先保证接口和数据格式稳定。

## 6/16 周二：P1 曲线和 Fast/Safe mode

### 1. 速度模式

文件：`src/control/controller.py`

C 会在 `JointTrajectory.speed_profile` 里给 `fast` 或 `safe`。

你要做：fast 用正常速度；safe 用较慢速度；在日志里能看出不同阶段执行时间或速度不同。

### 2. 避障距离

文件：`src/control/logger.py`

要记录：每个轨迹点到最近障碍物的距离；最小 obstacle clearance。如果距离由 C 计算，D 负责保存和画图。

### 3. 曲线数据

文件：`src/visualization/plot_results.py`

函数：

```python
def build_plot_data(execution: ExecutionResult) -> dict[str, list[float]]:
```

必须包含：`joint_error`、`end_effector_error`、`obstacle_clearance`。

可以后续再加真正 matplotlib 保存 png 的函数：

```python
def save_plots(execution: ExecutionResult, output_dir: str = "results") -> list[str]:
```

新增函数前先通知 C 更新接口文档。

## 6/18 周四：P2 reset 和人手暂停结果

要输出这些场景的结果：普通走子、吃子、`obstacle_mode 1/2/3` 绕障对比、reset、hand_on 后 safe / pause、hand_off 继续。

建议在 `results/` 保存：

- `joint_error_demo.csv`
- `end_effector_error_demo.csv`
- `obstacle_clearance_demo.csv`
- `summary_table.csv`
- 对应 png 曲线

summary 表格建议包含：

- obstacle mode；
- 是否出现 hand_on；
- safety decision：continue / safe / pause；
- 最大 joint error；
- 最大 end-effector error；
- 最小 obstacle clearance；
- execution time。

## 6/21 周日：PPT 验证页

你要给 A 的 PPT 提供：controller design 简图或文字；joint tracking error 曲线；end-effector position error 曲线；obstacle clearance 曲线；表格：每个 demo 是否成功、最大误差、最小避障距离、执行时间。

## 提交前检查

```bash
python -m unittest discover -s tests -v
python main.py --demo
```

不要改：`src/interaction/`、`src/simulation/`、`src/planning/`、`src/common/`、`main.py`。如果需要新增日志字段，先找 C 改 `ExecutionResult`。
