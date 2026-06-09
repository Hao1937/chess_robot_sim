# Chess Robot Simulation

基于仿真交互与动态避障的象棋机械臂系统。

本目录是 6/9 架构冻结版本，目标是让四个成员按稳定接口并行开发：

- A：`src/interaction/`，负责输入、棋盘状态、棋子规则、reset 指令。
- B：`src/simulation/`，负责 PyBullet 场景、机械臂、棋子、虚拟磁吸附。
- C：`src/common/`、`src/planning/`、`main.py`、接口文档，负责技术架构、坐标、IK、轨迹规划、集成。
- D：`src/control/`、`src/visualization/`，负责控制执行、日志、曲线和验证结果。

## 快速运行

当前骨架不依赖 PyBullet 即可运行 mock pipeline：

```bash
python main.py --demo
```

在仓库根目录运行测试：

```bash
python -m unittest discover -s tests -v
```

## 重要约定

- `main.py`、`src/common/`、`docs/interface_contract.md` 由 C 统一维护。
- 其他成员尽量只改自己负责的目录。
- 如果需要新增接口或改数据结构，先和 C 对齐，再改 `src/common/types.py` 和接口文档。
- 不做象棋 AI、不做真实视觉识别、不做真实夹爪动力学、不做移动棋盘。
