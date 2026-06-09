# 成员 D TODO：控制器与实验验证

你负责 `src/control/` 和 `src/visualization/`，目标是让机械臂轨迹可执行，并输出能放进 PPT 的验证结果。

## 你负责的文件

- `src/control/controller.py`
- `src/control/logger.py`
- `src/visualization/plot_results.py`
- `results/`
- PPT 验证页素材

## 起手任务

1. 阅读 `docs/interface_contract.md` 中 D 的接口。
2. 在 `execute_trajectory()` 中接收 C 输出的 `JointTrajectory`。
3. 实现 joint-space position control 或 PID control。
4. 使用 `speed_profile` 实现 Fast/Safe mode：
   - fast：正常速度；
   - safe：靠近障碍物、棋子或低空阶段减速。
5. 记录：
   - desired joint angle；
   - actual joint angle；
   - joint tracking error；
   - end-effector position error；
   - obstacle clearance；
   - execution time；
   - success rate。
6. 在 `plot_results.py` 中输出曲线：
   - joint tracking；
   - joint error；
   - end-effector error；
   - obstacle clearance。
7. 为这些 demo 整理结果：
   - 普通走子；
   - 吃子；
   - 绕障；
   - reset；
   - 人手暂停。

## 不要做

- 不要写棋盘规则。
- 不要写 PyBullet 场景。
- 不要直接改规划器接口。
- 不要直接改 `main.py`。

## 验收标准

- `execute_trajectory()` 能被 `main.py` 调用。
- `summarize_execution()` 能返回最大误差、最小避障距离和执行时间。
- 能给 PPT 提供曲线图和结果表。
