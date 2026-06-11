# Interaction Spec

本文件由成员 A 维护，说明用户输入如何变成 `MoveCommand`。

## CLI Commands

| 命令 | 含义 |
|---|---|
| `A1 B1` | 从 `A1` 移动到 `B1` |
| `reset` | 清盘 / 复位 |
| `hand_on` | 模拟人手进入安全区 |
| `hand_off` | 模拟人手离开安全区 |
| `obstacle_mode 1` | 使用第 1 组预设圆柱障碍 |
| `obstacle_mode 2` | 使用第 2 组预设圆柱障碍 |
| `obstacle_mode 3` | 使用第 3 组预设圆柱障碍 |
| `quit` / `exit` | interactive 模式下退出会话 |
| `车二平七` | 红方中文记谱：二路线红车平移到七路 |
| `炮五进四` | 红方中文记谱：五路红炮前进四格 |

## Chinese Notation

中文记谱只作为高级输入功能，不是象棋 AI。

当前只支持红方，并且只支持两类规则：

- `车二平七`：找到红方二路车，横向移动到七路。
- `炮五进四`：找到红方五路炮，向前移动四格。

约定：

- 一到九直接映射为 A 到 I；
- 红方前进表示行号增加；
- 如果同一路有多个同类红方棋子，必须让玩家输入 `前` 或 `后`，例如 `前车二平七`；
- 不支持黑方中文记谱；
- 不支持完整象棋裁判和 AI。

## GUI / Interactive 接口

GUI 不是一次性函数。它应该长期存在，每次用户操作只吐出一条命令。

A 对外提供的轮询接口是：

```python
def poll_gui_command(board: BoardState, input_func=input, prompt: str = "move> ") -> MoveCommand | None:
```

约定：

- 用户没有输入或没有新事件时，返回 `None`；
- 用户输入 `quit` / `exit` 时，返回 `MoveCommand(command_type="quit")`；
- 用户输入坐标命令时，复用 `parse_command()`；
- 用户输入中文记谱时，复用 `parse_chinese_move(text, board)`；
- 真 GUI 做好后，也保持“一次用户事件返回一条 `MoveCommand`”这个接口。

主循环由 C 的 `run_interactive()` 负责，不由 A 在 GUI 里自己跑完整 pipeline。

## GUI Minimum Version

GUI 最小版本只需要：

1. 起点输入；
2. 终点输入；
3. submit 按钮；
4. reset 按钮；
5. hand_on / hand_off 按钮；
6. obstacle mode 选择控件，至少支持 1/2/3 三组预设障碍；
7. 中文记谱输入框，可输入 `车二平七` 或 `炮五进四`；
8. quit / exit 或关闭窗口时能通知主循环退出；
9. 后续可扩展完整棋盘点击界面。
