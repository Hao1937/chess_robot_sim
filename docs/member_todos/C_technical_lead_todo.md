# 成员 C TODO：技术负责人、接口、坐标、IK、规划、集成

你负责 `main.py`、`src/common/`、`src/planning/` 和接口文档。

你的模块回答一个问题：**A/B/D 的模块怎么接起来，逻辑动作怎么变成机械臂可执行轨迹？**

C 是技术负责人。任何接口、坐标系、数据结构、主流程修改，都由 C 统一维护。

## 你负责的文件

| 文件 | 你要做什么 |
|---|---|
| `main.py` | 主流程集成 |
| `src/common/types.py` | 统一数据结构 |
| `src/common/config.py` | 统一坐标、尺寸、速度参数 |
| `src/planning/chessboard_mapping.py` | 棋盘格到世界坐标 |
| `src/planning/ik_solver.py` | IK 和可达性检查 |
| `src/planning/motion_primitives.py` | 逻辑动作到运动基元 |
| `src/planning/obstacle_map.py` | 障碍物地图和动态障碍介入判断 |
| `src/planning/trajectory_planner.py` | waypoint、轨迹、速度模式 |
| `docs/interface_contract.md` | 接口契约 |
| `tests/test_contract_interfaces.py` | 接口测试 |

## 6/9 周二：架构冻结

必须保证：`main.py --demo` 能跑通 mock pipeline；`docs/interface_contract.md` 写清楚每个模块的函数和输入输出；A/B/D 的 TODO 文件写清楚改什么文件、写什么函数；`src/common/types.py` 和 `src/common/config.py` 不频繁变。

验收：

```bash
python -m unittest discover -s tests -v
python main.py --demo
```

## 6/11 周四：坐标映射和 IK 起步

### 1. 棋盘坐标

文件：`src/planning/chessboard_mapping.py`

函数：

```python
def cell_to_world(cell: str, config: Config = DEFAULT_CONFIG) -> tuple[float, float, float]:

def cell_above_world(cell: str, config: Config = DEFAULT_CONFIG) -> tuple[float, float, float]:
```

要求：`A1` 是棋盘左下角；`cell_size` 从 `Config` 读；`CAPTURED_*` 也能映射到 captured area；B 的棋盘显示必须和这里一致。

### 2. IK 接口

文件：`src/planning/ik_solver.py`

函数：

```python
def solve_ik(target_xyz: tuple[float, float, float], config: Config = DEFAULT_CONFIG) -> tuple[float, ...]:

def is_reachable(target_xyz: tuple[float, float, float], config: Config = DEFAULT_CONFIG) -> bool:
```

要求：先保留 mock 也可以，但签名不能变；B 加载真实机械臂后，替换为 PyBullet IK；不可达时要让 A/GUI 能得到错误提示。

## 6/14 周日：P0 集成

### 1. 运动基元

文件：`src/planning/motion_primitives.py`

函数：

```python
def build_motion_primitives(actions: list[LogicalAction], config: Config = DEFAULT_CONFIG) -> list[MotionPrimitive]:
```

pick 拆成：`approach` 到棋子上方 fast；`descend` 下降抓取 safe；`lift` 抬升到 safe height safe。

place 拆成：`transfer` 高空转移 fast 或 safe；`descend` 下降放置 safe；`retreat` 抬升 safe。

reset 不另起特殊系统，直接复用 pick/place 序列。

### 2. 主流程集成

文件：`main.py`

主流程必须是：B `load_robot()` 和 `build_scene()`；A `parse_command()`、`validate_move()`、`make_logical_actions()`；C `build_motion_primitives()`、`build_obstacle_map()`、`plan_trajectory()`；B `attach_piece()` / `detach_piece()`；D `execute_trajectory()` 和 `summarize_execution()`。

不要把 A/B/D 的细节写进 `main.py`。

## 6/16 周二：P1 避障和 Fast/Safe mode

### 1. 障碍地图

文件：`src/planning/obstacle_map.py`

函数：

```python
def build_obstacle_map(
    piece_cells: list[str],
    extra_obstacles: list[Obstacle],
    human_hand_present: bool = False,
    config: Config = DEFAULT_CONFIG,
) -> list[Obstacle]:
```

要求：棋子转成 inflated obstacle；obstacle mode 的预设圆柱加入 obstacle list；hand_on 时加入 human hand dynamic obstacle；正在抓取的棋子后续可以从 obstacle list 中排除。

同时提供动态障碍介入判断：

```python
def assess_obstacle_intervention(
    target_xyz: tuple[float, float, float],
    obstacles: list[Obstacle],
    config: Config = DEFAULT_CONFIG,
) -> SafetyDecision:
```

返回：

- `continue`：目标仍然可达且远离动态障碍；
- `safe`：目标可达但靠近动态障碍，进入慢速安全模式；
- `pause`：目标不可达或被人手区域阻挡，提示移除障碍。

### 2. 轨迹规划

文件：`src/planning/trajectory_planner.py`

函数：

```python
def plan_trajectory(
    primitives: list[MotionPrimitive],
    obstacles: list[Obstacle],
    config: Config = DEFAULT_CONFIG,
) -> JointTrajectory:
```

要求：每个 primitive 通过 IK 转为关节 waypoint；接近障碍物时 speed profile 输出 `safe`；无障碍高空转移可以输出 `fast`；低空 descend / retreat 必须 `safe`；后续可以加入绕行 waypoint。

## 6/18 周四：P2 reset、规则、GUI、人手区集成

你要负责把 A/B/D 的 P2 功能接入主流程：`reset` 命令能从 A 到 C 到 B/D 跑通；`obstacle_mode` 能切换 B 的预设圆柱障碍；`hand_on` 能进入 obstacle map 并调用 `assess_obstacle_intervention()`；`hand_off` 后能恢复执行或重新规划；GUI 输出的 `MoveCommand` 和 CLI 一致。

如果需要新增共享类型，只能你改：`src/common/types.py`、`docs/interface_contract.md`、`tests/test_contract_interfaces.py`。

## 6/21 周日：最终集成

最终你要保证：`main.py --demo` 能跑；demo 视频中的每一步都有代码路径支撑；A/B/D 没有人绕过接口直接互相调用内部细节；README 写清楚运行方法。

## 提交前检查

```bash
python -m unittest discover -s tests -v
python main.py --demo
python -m compileall .
```

## Interactive 主流程接口

C 负责 `main.py` 里的长期会话入口：

```python
def run_command(command, board, scene, robot, config=DEFAULT_CONFIG) -> dict[str, object]:

def run_interactive(input_func=input, output_func=print, config=DEFAULT_CONFIG, max_steps=None) -> dict[str, object]:
```

要求：

- `run_interactive()` 只初始化一次 `BoardState`、robot、scene；
- 每轮调用 A 的 `poll_gui_command()` 获取一条命令；
- 命令成功执行后调用 `apply_logical_actions()` 更新内存棋盘状态；
- `quit` / `exit` 退出会话；
- 不要让 A/B/D 自己复制主循环。
