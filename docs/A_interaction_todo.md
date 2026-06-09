# 成员 A TODO：交互、棋盘状态、棋子规则

你负责目录：`src/interaction/`。

你的模块回答一个问题：**用户输入了一步棋，系统应该生成什么逻辑动作？**

不要写机器人运动、IK、PyBullet 控制。你只输出 `MoveCommand`、`ValidationResult`、`LogicalAction`，后面由 C/B/D 接。

## 你负责的文件

| 文件 | 你要做什么 |
|---|---|
| `src/interaction/cli.py` | 解析命令行输入 |
| `src/interaction/board_state.py` | 维护棋盘状态，生成逻辑动作序列 |
| `src/interaction/chess_rules.py` | 判断简化象棋走法是否合法 |
| `src/interaction/gui.py` | GUI 输入和 hand_on/hand_off 命令适配 |
| `docs/interaction_spec.md` | 记录交互命令格式和 GUI 最小需求 |

## 必须遵守的接口

从 `src/common/types.py` 使用这些类型：

- `MoveCommand`
- `BoardState`
- `Piece`
- `PieceType`
- `PieceColor`
- `ValidationResult`
- `LogicalAction`

你输出的动作必须是 `list[LogicalAction]`，不要自己发明新格式。

## 6/11 周四：模块起步

### 1. 完善 `parse_command()`

文件：`src/interaction/cli.py`

函数：

```python
def parse_command(command_text: str) -> MoveCommand:
```

要支持：

- `A1 B1`：移动命令，返回 `MoveCommand(command_type="move", from_cell="A1", to_cell="B1")`
- `reset`：复位命令，返回 `MoveCommand(command_type="reset")`
- `hand_on`：人手进入，返回 `MoveCommand(command_type="hand_on")`
- `hand_off`：人手离开，返回 `MoveCommand(command_type="hand_off")`

验收：空输入要报错；`A1` 这种只有一个格子的输入要报错；大小写不敏感，`a1 b1` 应转成 `A1 B1`。

### 2. 完善 `create_initial_board()`

文件：`src/interaction/board_state.py`

函数：

```python
def create_initial_board() -> BoardState:
```

要做：至少保留 demo 棋子 `A1` 红车、`B1` 黑马、`C1` 红炮；可以逐步加入更多初始棋子；每个棋子必须有唯一 `piece_id`。

### 3. 完善 `make_logical_actions()`

文件：`src/interaction/board_state.py`

函数：

```python
def make_logical_actions(board: BoardState, command: MoveCommand) -> list[LogicalAction]:
```

普通移动输出：pick 起点，place 终点。

吃子输出：pick 敌方棋子，place 到 `CAPTURED_BLACK_1` / `CAPTURED_RED_1`，再 pick 己方棋子，place 到目标格。

reset 输出：遍历当前棋盘，如果棋子不在初始位置，输出 pick/place 回初始位置。

## 6/14 周日：P0 集成

你要保证：

- `A1 B1` 能生成吃子动作序列。
- `A1 C1` 如果目标是己方棋子，要返回非法。
- `reset` 能返回一组 reset 逻辑动作。
- 你的代码不改 `main.py`，由 C 接入。

C 会调用：

- `parse_command()`
- `validate_move()`
- `make_logical_actions()`

## 6/16 周二：P1 功能

### 1. demo 命令表

在 `docs/interaction_spec.md` 里写清楚：普通走子 demo、吃子 demo、绕障 demo、reset demo、`hand_on` / `hand_off` demo。

### 2. 棋盘状态更新接口

建议补函数：

```python
def apply_logical_actions(board: BoardState, actions: list[LogicalAction]) -> BoardState:
```

作用：执行 place 后更新棋子所在 cell；被吃棋子进入 `CAPTURED_*`；reset 后棋子回初始位置。

如果新增这个函数，先告诉 C，让 C 更新接口文档和测试。

## 6/18 周四：P2 功能

文件：`src/interaction/chess_rules.py`

必须完成：车同行或同列；马走日字；炮同行/同列，普通移动中间 0 个棋子，吃子中间 1 个棋子。

文件：`src/interaction/gui.py`

最小目标：能构造 `MoveCommand`；能构造 `hand_on` / `hand_off`；GUI 可以很简陋，可以先用输入框和按钮。

不需要做：将军判断、胜负判断、完整象棋 AI、全部棋子的复杂限制。

## 提交前检查

```bash
python -m unittest discover -s tests -v
python main.py --demo
```

不要改：`main.py`、`src/common/types.py`、`src/common/config.py`、`docs/interface_contract.md`。如果确实需要改，先找 C。
