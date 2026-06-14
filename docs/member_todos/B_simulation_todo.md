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

## 棋盘和棋子可视化范围

B 的可视化做到第二版为止：重点是让仿真场景清楚、稳定、能和 A/C/D 接口对上，不做精细美术建模。

### 第一版必须完成

- 棋盘用 PyBullet box primitive 建一个薄平板，不需要单独建 URDF。
- 棋盘尺寸覆盖 9 列 x 10 行，位置和 `DEFAULT_CONFIG.board_origin`、`cell_size` 对齐。
- 棋子用 cylinder primitive 表示，不需要真实象棋模型。
- 红方棋子用红色，黑方棋子用深色或黑色。
- 静态障碍物和 obstacle mode 也用 cylinder primitive。
- 坐标转换不要自己写一套，棋子落点必须和 C 的 `cell_to_world(cell, config)` 保持一致。


### 尺寸来源

B 不要自己硬编码棋盘和棋子尺寸，统一从 `src/common/config.py` 的 `Config` 读取：

```python
board_cols = config.board_cols      # 默认 9
board_rows = config.board_rows      # 默认 10
cell_size = config.cell_size        # 默认 0.06 m
piece_radius = config.piece_radius  # 默认 0.0225 m
piece_height = config.piece_height  # 默认 0.018 m
```

棋盘平板可以按下面方式推导：

```python
board_width = config.board_cols * config.cell_size   # 默认 0.54 m
board_depth = config.board_rows * config.cell_size   # 默认 0.60 m
```

棋子可视化可以用 cylinder primitive：半径用 `config.piece_radius`，高度用 `config.piece_height`。C 做避障时会使用 `config.inflated_piece_radius`，B 不要把 inflated radius 当成棋子的真实显示半径。

### 第二版必须完成

- 给棋子顶部或旁边加文字标签，例如 `车`、`炮`，让演示时能看出是什么棋子。
- 在棋盘旁边画出 captured area，也就是吃子区。
- 吃子时能看出目标格敌方棋子先进入 captured area，然后己方棋子落到目标格。
- reset 时，棋子能回到初始位置；captured area 里的棋子也能被移回原位。
- `attach_piece()` / `detach_piece()` 要让棋子视觉上跟随机械臂末端移动，可以用 PyBullet constraint，也可以先用简化位置更新。
- 固定一个适合录屏的 camera view，能同时看到机械臂、棋盘、障碍物和吃子区。

### 明确不做

- 不做精细棋盘 3D 建模。
- 不做真实木纹、棋子雕刻、复杂材质。
- 不要求棋子贴图。
- 不要求每个棋子做独立 URDF 或 mesh。
- 不负责判断象棋规则，规则归 A。
- 不负责 IK、路径规划和避障决策，规划归 C。
- 不负责关节控制器和误差图，控制验证归 D。

## 6/11 周四：模块起步

### 1. 机械臂加载

文件：`src/simulation/load_robot.py`

要做：

- 选定了UR5模型
- 用 PyBullet 加载 URDF。
- 返回真实 `RobotHandle`：`robot_id`、`end_effector_id`、`joint_indices`。

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

### 1. reset 场景响应

当 A/C 给出 reset 对应的 pick/place 动作序列后，B 只需要保证仿真场景能正确响应这些动作：

- `attach_piece()` 后，棋子能跟随机械臂末端移动；
- `detach_piece()` 后，棋子能停在目标位置；
- 如果目标位置是初始格，棋子视觉上回到初始格；
- 如果棋子之前在 captured area，也能通过同样的 pick/place 流程被移回。

B 不需要自己判断 reset 应该怎么走，也不需要自己决定棋子的初始位置；reset 的逻辑动作由 A/C 生成。

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


