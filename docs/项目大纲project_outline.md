# 项目大纲：交互式象棋机械臂仿真系统

这份文档用直观语言说明我们到底要做什么、用什么技术栈、最后能展示什么，以及一些容易产生歧义的细节。它用于确认项目方向和项目结构是否一致。

## 1. 一句话概括

我们要做一个 **仿真里的象棋机械臂系统**：用户输入或点击一步象棋走法，系统判断棋盘状态，然后机械臂在 PyBullet 里完成取子、移动、吃子、避障、落子、复位，并输出轨迹和控制误差结果。

这个项目重点不是象棋 AI，而是 robotics/control 相关内容：

- 机械臂模型加载；
- 棋盘坐标映射；
- pick-and-place；
- 吃子任务编排；
- safe height 避障；
- 障碍物和人手安全区处理；
- 轨迹规划和速度模式；
- 控制执行和误差验证。`n- 可选高级输入：红方中文记谱，例如 `车二平七`、`炮五进四`。

## 2. 我们不做什么

为了避免项目范围失控，以下内容明确不做：

- 不做象棋 AI，不让程序自动思考下一步棋。
- 不做完整象棋裁判系统，不做将军、将死、胜负判断。
- 不做真实视觉识别，不从摄像头识别棋盘。
- 不做真实夹爪动力学，棋子抓取用**虚拟磁吸附**或 suction-like attachment。
- 不做移动棋盘、旋转棋盘或在线跟踪移动目标。
- 不做真实机器人，只做 simulation validation（仿真验证）。

## 3. 用户看到的功能

最终用户可以做这些事：

1. 输入一步棋，例如 `A1 B1`。（前期用cli输入命令，后期用GUI点击平面棋盘实现）
2. 系统判断起点有没有棋子，目标格有没有棋子。
3. 如果目标格为空，机械臂直接移动棋子。
4. 如果目标格有敌方棋子，机械臂先把敌方棋子移到 captured area（吃子区），再把己方棋子放到目标格。
5. 用户可以触发 `reset`，让机械臂把棋子放回初始位置或指定区域。
6. 用户可以触发 `hand_on`，模拟人手进入棋盘区域，机械臂重新规划路径，根据目标是否仍然可达进入暂停（输出不可达信号并提示移除障碍）或安全模式（缓慢运动）。
7. 用户触发 `hand_off` 后，机械臂继续执行或重新规划。
8. 最后可以看到误差曲线、避障距离曲线和 demo 视频。
9. 用户可以切换 `obstacle_mode 1/2/3`，场景中出现不同位置的竖直圆柱障碍，用来测试避障规划。`n10. 用户可以输入少量红方中文记谱，例如 `车二平七`、`炮五进四`，系统把它转换成坐标走法。

## 4. 技术栈

项目优先使用：

- Python：主语言；
- PyBullet：机械臂和棋盘仿真；
- URDF：机械臂模型描述；
- NumPy：后续做轨迹和数值计算；
- Matplotlib：后续画误差曲线；
- unittest：基础接口测试。

机械臂模型优先使用 UR5。当前模型资源放在：

```text
assets/ur5/
├── ur5_robot.urdf
├── ur5_joint_limited_robot.urdf
├── visual/
└── collision/
```

URDF 里的 mesh 路径统一使用同目录下的：

```text
visual/*.dae
collision/*.stl
```

## 5. 项目结构如何对应功能

```text
main.py
src/common/
src/interaction/
src/simulation/
src/planning/
src/control/
src/visualization/
assets/
results/
docs/
tests/
```

各目录含义：

- `main.py`：主流程，只由 C 维护；会把 `obstacle_mode` 命令传给场景构建。
- `src/common/`：共享类型和配置，只由 C 维护。
- `src/interaction/`：A 负责，处理用户输入、中文记谱、棋盘状态、走法规则、reset、GUI 命令。
- `src/simulation/`：B 负责，处理 PyBullet、URDF、棋盘棋子场景、虚拟吸附、人手安全区可视化。
- `src/planning/`：C 负责，处理坐标映射、IK、运动基元、障碍地图、轨迹规划、Fast/Safe mode。
- `src/control/`：D 负责，处理轨迹执行、关节控制、误差记录。
- `src/visualization/`：D 负责，处理曲线数据和结果图。
- `assets/`：模型、mesh、贴图等资源。
- `results/`：曲线、csv、截图、验证结果。
- `docs/`：接口文档、任务文档、项目说明。
- `tests/`：接口测试。

## 6. 系统主流程

一次普通走子的流程是：

1. 用户输入 `A1 B1`。
2. A 的 `parse_command()` 把输入变成 `MoveCommand`。
3. A 的 `validate_move()` 判断走法是否合法。
4. A 的 `make_logical_actions()` 生成逻辑动作，例如 pick A1、place B1。
5. C 的 `build_motion_primitives()` 把逻辑动作拆成机器人动作，例如 approach、descend、lift、transfer、place。
6. C 的 `build_obstacle_map()` 根据棋子、障碍柱、人手状态生成障碍物列表。
7. C 的 `plan_trajectory()` 生成关节轨迹和速度模式。
8. B 的 `attach_piece()` / `detach_piece()` 负责棋子吸附和释放的仿真表现。
9. D 的 `execute_trajectory()` 执行轨迹。
10. D 的 `summarize_execution()` 和 `plot_results()` 输出误差和曲线数据。`n`n如果用户输入的是 `车二平七` 或 `炮五进四`，A 先用 `parse_chinese_move()` 根据当前棋盘找到对应红方棋子，再转换成同样的 `MoveCommand`，后续流程不变。

## 7. 吃子功能怎么做

吃子不是让棋子直接覆盖目标格，而是拆成两段任务：

1. 先抓取目标格上的敌方棋子；
2. 把敌方棋子放到 captured area；
3. 再抓取起点的己方棋子；
4. 把己方棋子放到目标格。

对应逻辑动作类似：

```python
[
    pick B1,
    place CAPTURED_BLACK_1,
    pick A1,
    place B1,
]
```

这样视频里能清楚看到“先移走被吃棋子，再落子”的任务逻辑。

## 8. reset 清盘 / 复位怎么做

reset 不单独设计复杂算法，而是复用 pick-and-place。

A 负责根据棋盘状态生成 reset 动作序列：

- 哪个棋子不在初始位置；
- 它应该回到哪个格子；
- 输出一串 pick/place 逻辑动作。

C 把这些动作转成 motion primitives。

B 在仿真中更新棋子位置。

D 记录 reset 的执行时间和误差。

也就是说，reset 本质上是很多次普通移动任务的组合。

## 9. 不同棋子走法怎么做

我们只做简化规则，不做完整象棋裁判。

由于不需要做完整棋盘规则，我们这个项目只做三个：

- 车：只允许同行或同列移动。
- 马：走日字，也就是列差和行差为 `(1,2)` 或 `(2,1)`。
- 炮：普通移动类似车；吃子时中间必须隔一个棋子。

其他棋子只做基础合法性检查，例如起点有棋子、目标格不是己方棋子。

这部分由 A 的 `src/interaction/chess_rules.py` 负责。


## 9.1 中文记谱输入怎么做

中文记谱只做红方，而且只做两个亮点输入：

- `车二平七`：二路红车横向平移到七路；
- `炮五进四`：五路红炮向前进四格。

这里的一到九采用项目内简化约定：一到九对应 A 到 I。红方前进表示棋盘行号增加。

如果同一路上存在多个同类红方棋子，系统不猜测，而是提示玩家加 `前` 或 `后` 来确定，例如：

```text
前车二平七
后车二平七
```

这只是输入解析功能，不是象棋 AI。
## 10. 避障怎么做

避障分三层。

第一层：safe height。

机械臂不在棋盘低空横扫，而是：

1. 到棋子上方；
2. 垂直下降；
3. 吸附棋子；
4. 抬升到 safe height；
5. 高空转移；
6. 到目标格上方；
7. 垂直下降放子。

第二层：棋子和障碍柱作为 inflated obstacles。

C 会把棋子、obstacle mode 的预设圆柱、人手安全区都变成带安全半径的障碍物。路径接近这些障碍时，轨迹 planner 进入更保守模式。

第三层：绕行 waypoint。

如果高空转移路径太靠近障碍物，可以插入额外 waypoint，让路径绕开障碍物。

## 11. Fast/Safe mode 是什么

Fast/Safe mode 是速度策略。

- Fast mode：路径远离障碍物，而且处在高空转移阶段，可以正常速度移动。
- Safe mode：靠近棋子、障碍物、人手安全区，或者处在下降/放置/抬升阶段，就减速。

C 在 `JointTrajectory.speed_profile` 里输出 `fast` 或 `safe`。

D 的 controller 根据 speed profile 改变执行速度。

最终展示时可以说：机器人会根据环境风险自动切换速度模式。

## 12. 人手障碍介入怎么做

人手不是通过真实摄像头检测，而是通过 GUI 或命令模拟。

有两种触发方式：

```text
hand_on
hand_off
```

或 GUI 上两个按钮：

```text
Hand Enter
Hand Leave
```

人手进入时的系统行为：

1. A 生成 `MoveCommand(command_type="hand_on")`。
2. B 在仿真场景里显示一个半透明红色区域或动态障碍块，代表人手安全区。
3. C 的 `build_obstacle_map()` 把人手区域加入 obstacle list。
4. C 调用 `assess_obstacle_intervention()` 判断目标是否仍可达。
5. 如果目标被人手区域挡住或不可达，系统 pause，并提示移除障碍。
6. 如果目标仍可达但靠近人手区域，系统进入 Safe mode，缓慢运动。
7. 如果目标远离人手区域，系统继续执行或重新规划。
8. D 记录暂停时间、执行状态或安全模式切换。

人手离开时：

1. A 生成 `MoveCommand(command_type="hand_off")`。
2. B 隐藏或禁用人手障碍。
3. C 移除 human hand obstacle。
4. 机械臂继续执行剩余轨迹，必要时重新规划。

最低实现要求：**hand_on 暂停，hand_off 继续**。

最好能够实现：hand_on 后重新规划绕开人手区域。

## 13. obstacle mode 怎么做

`obstacle_mode` 是专门用于测试避障规划的静态障碍预设。

命令形式：

```text
obstacle_mode 1
obstacle_mode 2
obstacle_mode 3
```

每个模式对应 2-3 个不同位置的竖直圆柱体。它们不是棋子，也不是人手，而是固定障碍物，用来做 without avoidance vs with avoidance、Fast/Safe mode 和 obstacle clearance 曲线。

B 在 `build_scene(config, obstacle_mode)` 中生成这些圆柱障碍。

C 在 `build_obstacle_map()` 中把这些障碍纳入 inflated obstacles。

D 在结果里记录当前 obstacle mode、最小避障距离和执行时间。

## 14. GUI 做到什么程度

GUI 不需要一开始很复杂。

最低版本只需要：

- 起点输入框；
- 终点输入框；
- submit 按钮；
- reset 按钮；
- hand_on 按钮；
- hand_off 按钮。
- obstacle mode 选择控件。

如果时间够，再做点击棋盘格子。

GUI 的输出必须和 CLI 一样，都是 `MoveCommand`。这样后面的规划和控制不用关心输入来自 GUI 还是命令行。

## 15. 最终能展示什么

最终 demo 目标：

1. 机械臂和棋盘在 PyBullet 中显示。
2. 用户输入一步棋。
3. 机械臂移动棋子。
4. 机械臂执行吃子：先移走敌方棋子，再落子。
5. 路径避开棋子或障碍柱。
6. 靠近障碍物或低空操作时自动减速。
7. reset 后棋子能回到初始位置或指定区域。
8. hand_on 后机械臂暂停或进入安全模式。
9. hand_off 后机械臂继续。
10. 最后展示关节误差、末端误差、避障距离和执行时间。
11. 切换不同 obstacle mode 后，展示路径和避障距离变化。`n12. 输入 `车二平七` 或 `炮五进四` 后，系统能转换为坐标走法并执行。

## 16. 最终视频建议

视频可以按这个顺序录：

1. 先展示最终效果：输入一步棋，机械臂移动棋子。
2. 展示吃子：先移走目标棋子，再移动己方棋子。
3. 展示绕障：有障碍时路径变化。
4. 展示 obstacle mode 1/2/3：不同圆柱障碍位置下的路径变化。
5. 展示 Fast/Safe mode：靠近棋子或障碍物时减速。
6. 展示 reset。
7. 展示 hand_on / hand_off。
8. 展示误差曲线和结果表。

## 17. 验收标准

最低验收：

```bash
python main.py --demo
python -m unittest discover -s tests -v
```

功能验收：

- 普通移动能跑通；
- 吃子能跑通；
- reset 能生成并执行动作序列；
- 静态障碍物能显示；
- `obstacle_mode 1/2/3` 能切换预设圆柱障碍；
- 人手安全区能显示并触发暂停；
- controller 能输出误差数据；
- results 中有曲线或 csv；
- PPT 里能解释系统结构、避障策略、控制验证。

## 18. 项目最终形态

最终它不是一个“会下象棋的机器人”，而是一个 **能根据用户指令，在仿真环境中完成象棋任务的机械臂系统**。

我们展示的是机器人学系统能力：

- 从用户命令到任务序列；
- 从棋盘格到机器人坐标；
- 从动作序列到轨迹；
- 从轨迹到控制执行；
- 从执行结果到误差验证；
- 从动态障碍到安全响应。

