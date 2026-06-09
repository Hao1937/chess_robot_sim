# 成员 C TODO：技术负责人、项目架构与轨迹规划

你负责项目骨架、接口、主流程、坐标系、IK、轨迹规划和集成兜底。

## 你负责的文件

- `main.py`
- `src/common/config.py`
- `src/common/types.py`
- `src/planning/chessboard_mapping.py`
- `src/planning/ik_solver.py`
- `src/planning/motion_primitives.py`
- `src/planning/obstacle_map.py`
- `src/planning/trajectory_planner.py`
- `docs/interface_contract.md`

## 起手任务

1. 维护 repo 和项目结构。
2. 冻结接口：
   - 数据结构；
   - 函数签名；
   - 坐标系；
   - 主流程。
3. 实现棋盘坐标到世界坐标：
   - `A1` 为左下角；
   - `cell_size`、`z_board`、`z_safe` 全从 config 读取。
4. 实现 IK：
   - 先用 PyBullet IK；
   - 检查可达性；
   - 检查关节限制。
5. 实现 motion primitives：
   - pick；
   - lift；
   - transfer；
   - place；
   - reset；
   - safety_pause。
6. 实现 obstacle map：
   - 棋子 inflated obstacle；
   - 静态障碍柱；
   - human safety zone。
7. 实现 trajectory planner：
   - safe height；
   - waypoint；
   - 静态绕障；
   - 吃子路径；
   - reset 路径；
   - Fast/Safe mode。
8. 每次集成时检查 A/B/D 是否遵守接口。

## 不要做

- 不要承担 PPT 美化和剪视频。
- 不要替 A 写完整 GUI。
- 不要替 B 调全部场景材质。
- 不要替 D 画所有曲线。

## 验收标准

- `python main.py --demo` 能跑通主流程。
- A/B/D 可以在不改 `main.py` 的情况下接入自己的模块。
- 如果要改接口，先同步修改接口文档和测试。
