# 成员 A TODO：交互与棋局逻辑

你负责 `src/interaction/`，目标是把“用户输入”和“棋盘逻辑”转换成 C 能处理的 `LogicalAction`。

## 你负责的文件

- `src/interaction/cli.py`
- `src/interaction/board_state.py`
- `src/interaction/chess_rules.py`
- `src/interaction/gui.py`
- `docs/interaction_spec.md`

## 起手任务

1. 阅读 `docs/interface_contract.md` 中 A 的接口。
2. 保持 `parse_command()` 返回 `MoveCommand`。
3. 扩展 `create_initial_board()`，逐步放入更多象棋初始棋子。
4. 完善 `validate_move()`：
   - 车：直线；
   - 马：日字；
   - 炮：直线移动，吃子时中间隔一个棋子。
5. 完善 `make_logical_actions()`：
   - 普通走子：pick 起点，place 终点；
   - 吃子：先 pick 目标格敌方棋子，place 到 captured area，再移动己方棋子；
   - reset：生成一串复位动作。
6. GUI 可以先做简化版：
   - 输入起点/终点；
   - `hand_on` / `hand_off` 按钮；
   - 不要求一开始很好看。

## 不要做

- 不要写 IK。
- 不要写 PyBullet 运动。
- 不要直接改 `main.py`。
- 不要改 `src/common/types.py`，需要新字段先找 C。

## 验收标准

- `parse_command("A1 B1")` 能返回移动命令。
- `validate_move()` 能给出合法/非法和原因。
- 吃子能生成 4 个动作：pick enemy、place captured、pick self、place target。
- reset 能生成一组逻辑动作。
- 测试和 `main.py --demo` 不被你改坏。
