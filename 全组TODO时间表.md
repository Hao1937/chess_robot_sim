# 项目总 TODO：按 DDL 集成路线

本文件是全组总任务表。每个人的详细文件级任务见：

- A：`docs/member_todos/A_interaction_todo.md`
- B：`docs/member_todos/B_simulation_todo.md`
- C：`docs/member_todos/C_technical_lead_todo.md`
- D：`docs/member_todos/D_control_validation_todo.md`
- 接口契约：`docs/interface_contract.md`

项目原则：

- C 维护 `main.py`、`src/common/`、`docs/interface_contract.md`。
- A/B/D 不直接改 `main.py` 和 `src/common/`，需要改接口先找 C。
- 每个人优先改自己负责的目录。
- 提交前至少运行：`python -m unittest discover -s tests -v` 和 `python main.py --demo`。
- 不做象棋 AI、不做移动棋盘、不做真实视觉识别、不做真实夹爪动力学。
- 增加 `obstacle_mode 1/2/3`，用于切换 2-3 组预设竖直圆柱障碍，专门测试避障规划。
- 增加红方中文记谱高级输入，只支持 `车二平七`、`炮五进四`，并支持 `前` / `后` 消歧。

## 6/9 周二：架构冻结，只交 C

目标：让其他成员拿到项目后知道自己改什么文件、写什么函数、用什么数据结构。

C 必须完成：

- 建好项目结构：`src/common/`、`src/interaction/`、`src/simulation/`、`src/planning/`、`src/control/`、`src/visualization/`。
- 建好 `main.py` mock 主流程。
- 建好 `src/common/types.py`：统一 `MoveCommand`、`BoardState`、`LogicalAction`、`MotionPrimitive`、`Obstacle`、`JointTrajectory`、`ExecutionResult`。
- 建好 `src/common/config.py`：统一棋盘尺寸、格距、safe height、棋子半径、速度倍率。
- 建好 `docs/interface_contract.md`。
- 建好 A/B/C/D 的 TODO 文件。
- 确保 `python main.py --demo` 能跑通 mock pipeline。

A/B/D 当天只需要：

- 阅读 `docs/interface_contract.md`。
- 阅读自己的 TODO。
- 准备环境，不需要乱改接口。

## 6/11 周四：各模块按接口起步

目标：A/B/D 开始替换 mock，但不破坏主流程。

A 交付：

- 修改 `src/interaction/cli.py`：完善 `parse_command()`，支持 `obstacle_mode 1/2/3`。
- 修改 `src/interaction/board_state.py`：完善 `create_initial_board()` 和 `make_logical_actions()`。
- 修改 `src/interaction/chess_rules.py`：补车、马、炮的基础规则。
- 新增/修改 `src/interaction/chinese_notation.py`：支持红方 `车二平七`、`炮五进四`。
- 保证 `parse_command("A1 B1")`、吃子动作序列、reset 动作序列可用。

B 交付：

- 修改 `src/simulation/load_robot.py`：初步加载一个 URDF 机械臂，或至少确定模型路径和 joint indices。
- 修改 `src/simulation/scene_builder.py`：显示桌面、棋盘、棋子、吃子区、障碍柱，并支持 obstacle mode 预设。
- 修改 `src/simulation/attachment.py`：保留 `attach_piece()` / `detach_piece()` 接口。

D 交付：

- 修改 `src/control/controller.py`：让 `execute_trajectory()` 能接收 C 的 `JointTrajectory`。
- 修改 `src/control/logger.py`：让 `summarize_execution()` 输出最大误差、最小避障距离和执行时间。
- 修改 `src/visualization/plot_results.py`：准备曲线数据接口。

C 交付：

- 修改 `src/planning/chessboard_mapping.py`：确认棋盘坐标映射。
- 修改 `src/planning/ik_solver.py`：实现或准备 PyBullet IK 接口。
- 帮 A/B/D 接入，不让大家改散。

## 6/14 周日：P0 初版集成

目标：跑通主闭环：输入一步棋，机械臂完成 pick-and-place 和吃子初版。

必须跑通：

1. `python main.py --demo`。
2. CLI 输入 `A1 B1`。
3. A 生成 `LogicalAction`。
4. C 生成 `MotionPrimitive` 和 `JointTrajectory`。
5. B 显示 attach/detach 或对应占位。
6. D 输出 `ExecutionResult` 和 summary。
7. 吃子流程能执行：先移走目标棋子，再移动己方棋子。

验收命令：

```bash
python -m unittest discover -s tests -v
python main.py --demo
```

## 6/16 周二：P1 功能集成

目标：加入避障、轨迹可视化和 Fast/Safe mode。

A：提供普通走子、吃子、绕障 3 个 demo 命令；补充 `obstacle_mode 1/2/3` 命令；确认棋盘状态会随吃子和移动更新。

B：场景中支持 2-3 组预设竖直圆柱障碍；棋子能作为障碍物被 C 读取或转换；提供绕障录屏视角。

C：`obstacle_map.py` 生成棋子、预设障碍柱和人手区域的 inflated obstacles；`trajectory_planner.py` 在接近障碍物时输出 `safe`；低空阶段必须 `safe`；`assess_obstacle_intervention()` 判断 hand_on 后继续、安全慢速或暂停。

D：`controller.py` 根据 speed profile 调整速度；`logger.py` 记录 obstacle clearance；`plot_results.py` 输出 joint error、end-effector error、obstacle clearance 数据。

## 6/18 周四：P2 功能集成 + Beta 冻结

目标：加入 reset、简化棋子规则、GUI、人手安全区。

A：`reset` 指令能生成 reset 动作序列；车、马、炮规则可用；`gui.py` 能提供起点/终点输入或 hand_on/hand_off 控制。

B：reset 时仿真棋子位置能更新；human safety zone 可视化；人手安全区出现/消失能被场景表达。

C：reset 动作序列能转为 motion primitives；human safety zone 能进入 obstacle map；hand_on 时先更新障碍并评估目标是否仍可达，不可达则 pause，可达但接近障碍则 safe mode。

D：记录 reset 执行结果；记录 hand_on 暂停/继续结果；输出 PPT 用曲线和结果表。

Beta 冻结规则：6/18 后不再大改接口，只修 bug、补图、补视频、补报告。

## 6/21 周日：最终提交

最终交付：

- `README.md`
- `requirements.txt`
- `src/`
- `assets/`
- `results/`
- `demo_video.mp4`
- `presentation.pptx`
- `project_report.pdf` 或说明文档

最终 demo 顺序建议：普通走子、吃子、静态障碍绕行、Fast/Safe mode 减速、reset、GUI 人手安全区暂停、误差曲线和避障距离曲线。

