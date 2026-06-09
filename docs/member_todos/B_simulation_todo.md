# 成员 B TODO：PyBullet 仿真环境、场景、虚拟吸附

你负责目录：`src/simulation/` 和 `assets/`。

你的模块回答一个问题：**机器人、棋盘、棋子、障碍物在仿真里怎么显示和更新？**

不要写棋子规则、IK、轨迹规划、控制器。你只负责仿真对象和 attach/detach。

## 你负责的文件

| 文件 | 你要做什么 |
|---|---|
| `src/simulation/load_robot.py` | 加载机械臂 URDF，返回 robot handle |
| `src/simulation/scene_builder.py` | 创建桌面、棋盘、棋子、障碍物、人手安全区 |
| `src/simulation/attachment.py` | 实现虚拟磁吸附 attach/detach |
| `assets/` | 放 URDF、mesh、棋盘贴图或其他资源 |

## 必须遵守的接口

从 `src/common/types.py` 使用：`RobotHandle`、`SceneHandle`、`Obstacle`、`OperationResult`。

保持这些函数名不变：

```python
def load_robot(urdf_path: str | None = None) -> RobotHandle:

def build_scene(config: Config = DEFAULT_CONFIG, obstacle_mode: str = "mode_1") -> SceneHandle:

def attach_piece(piece_id: str, end_effector_id: int) -> OperationResult:

def detach_piece(piece_id: str) -> OperationResult:
```

## 6/11 周四：模块起步

### 1. 机械臂加载

文件：`src/simulation/load_robot.py`

要做：

- 选择一个模型：UR5 / UR5e / xArm6 / Panda / Lite6。
- 把模型资源放进 `assets/robot/`。
- 用 PyBullet 加载 URDF。
- 返回真实 `RobotHandle`：`robot_id`、`end_effector_id`、`joint_indices`。

如果暂时没找到模型：保留 mock 返回值，并在文件顶部注释候选模型路径和还缺什么。

### 2. 场景初版

文件：`src/simulation/scene_builder.py`

要创建：桌面、9 x 10 棋盘、至少 3 个 demo 棋子 A1/B1/C1、captured area、至少 1 个静态障碍柱。

同时实现障碍预设：

```python
def build_obstacle_preset(obstacle_mode: str, config: Config = DEFAULT_CONFIG) -> list[Obstacle]:
```

要求至少支持：

- `mode_1`：2 个竖直圆柱障碍；
- `mode_2`：3 个竖直圆柱障碍；
- `mode_3`：3 个不同位置的竖直圆柱障碍；
- `none`：无额外预设障碍。

返回：

```python
SceneHandle(
    board_id=..., 
    piece_ids={"A1": ..., "B1": ...},
    obstacles=[...]
)
```

注意：障碍物要转成 `Obstacle`，给 C 的 `build_obstacle_map()` 用；坐标不要自己另开体系，棋盘位置要和 C 的 `cell_to_world()` 对齐。

## 6/14 周日：P0 集成

### 1. 虚拟吸附

文件：`src/simulation/attachment.py`

实现：

- `attach_piece(piece_id, end_effector_id)`：机械臂到达棋子后，把棋子绑定到末端。
- `detach_piece(piece_id)`：到达目标后解除绑定。

可以先简化：用 PyBullet constraint，或先更新棋子位置，视觉上跟随末端。

保持返回：

```python
OperationResult(success=True, message="...")
```

### 2. 吃子显示

要能看出：先移动目标格敌方棋子到 captured area，再移动己方棋子到目标格。

## 6/16 周二：P1 功能

### 1. 障碍物可视化

文件：`src/simulation/scene_builder.py`

要做：静态障碍柱用明显颜色；不同 obstacle mode 的柱子位置明显不同；棋子颜色区分红黑；captured area 清楚可见；如果 C 输出轨迹点，能配合显示轨迹线或 waypoint 小球。

### 2. 绕障录屏视角

准备一个固定 camera view，方便录制普通走子、吃子、绕障。

## 6/18 周四：P2 功能

### 1. reset 场景更新

当 A/C 给出 reset 动作后，B 要保证：棋子回到初始位置；captured area 的棋子也能被移回。

### 2. human safety zone

文件：`src/simulation/scene_builder.py`

要做：显示一个半透明红色区域或动态障碍块；hand_on 时显示/启用；hand_off 时隐藏/禁用。这个人手障碍和 obstacle mode 的静态圆柱不同，它是 dynamic obstacle。

如果接口不够，先找 C 更新 `SceneHandle` 或新增函数。

## 提交前检查

```bash
python -m unittest discover -s tests -v
python main.py --demo
```

不要改：`src/interaction/`、`src/planning/`、`src/control/`、`src/common/`、`main.py`。如果需要新场景接口，先找 C。
