# Chess Robot Simulation

基于仿真交互与动态避障的中国象棋机械臂系统。（注意不是国际象棋！）

本目录目标是让四个成员按稳定接口并行开发：

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

## 需要阅读的文档

- ./docs/统一接口文档
- ./docs/项目大纲project_outline是项目的整体概念，防止理解有歧义
- ./docs/member_todos里是大家自己的任务

## 重要约定

- `main.py`、`src/common/`、`docs/interface_contract.md` 由 C 统一维护。
- 其他成员尽量只改自己负责的目录。
- 如果需要新增接口或改数据结构，先和 C 对齐，再改 `src/common/types.py` 和接口文档。
- 不做象棋 AI、不做真实视觉识别、不做真实夹爪动力学、不做移动棋盘。

## git的建议工作流

- **开始工作前先 pull，提交前先 test，只 add 自己负责的文件，冲突先沟通。**

## 推荐安装urdf-visualizer插件

在vscode安装后，可以直接预览urdf文件的模型

## 如何看可视化

```shell
python ./scripts/preview_simulation.py
```
