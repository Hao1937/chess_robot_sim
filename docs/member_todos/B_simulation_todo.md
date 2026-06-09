# 成员 B TODO：仿真环境与场景搭建

你负责 `src/simulation/`，目标是让系统在 PyBullet 里看得见、动得起来。

## 你负责的文件

- `src/simulation/load_robot.py`
- `src/simulation/scene_builder.py`
- `src/simulation/attachment.py`
- `assets/`
- demo 录屏素材

## 起手任务

1. 阅读 `docs/interface_contract.md` 中 B 的接口。
2. 在 `load_robot()` 中替换 mock：
   - 选择 UR5 / UR5e / xArm6 / Panda / Lite6 的 URDF；
   - 返回真实 robot id、end effector id、joint indices。
3. 在 `build_scene()` 中创建：
   - 桌面；
   - 9 x 10 棋盘；
   - 棋子圆柱体；
   - captured area；
   - 静态障碍柱；
   - human safety zone 可视化。
4. 在 `attachment.py` 中实现虚拟磁吸附：
   - 到达棋子后 attach；
   - 到达目标格后 detach。
5. reset 时支持棋子位置更新。
6. 录制素材：
   - 普通走子；
   - 吃子；
   - 绕障；
   - reset；
   - 人手安全区暂停。

## 不要做

- 不要写棋子走法规则。
- 不要写 IK 和轨迹规划。
- 不要直接改坐标系。
- 不要直接改 `main.py`，需要接入找 C。

## 验收标准

- PyBullet 能启动并显示机械臂。
- 棋盘、棋子、障碍物、吃子区可见。
- attach / detach 接口能被 `main.py` 调用。
- 场景对象 id 能通过 `SceneHandle` 返回。
