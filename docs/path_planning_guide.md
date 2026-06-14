# 轨迹规划与避障系统 — 技术深度指南

> **版本：** 2026-06-14  
> **模块路径：** `src/planning/`  
> **前置阅读：** `docs/统一接口文档.md`

---

## 目录

1. [问题场景与设计动机](#1-问题场景与设计动机)
2. [系统架构总览](#2-系统架构总览)
3. [碰撞检测 (Collision Checker)](#3-碰撞检测)
4. [路径搜索算法对比：A* vs RRT vs PRM](#4-路径搜索算法对比)
5. [A* on 2D Grid 实现细节](#5-a-on-2d-grid-实现细节)
6. [Waypoint 插值与轨迹平滑](#6-waypoint-插值与轨迹平滑)
7. [人手安全区精确建模](#7-人手安全区精确建模)
8. [集成流程与数据流](#8-集成流程与数据流)
9. [性能分析与参数调优](#9-性能分析与参数调优)
10. [扩展方向](#10-扩展方向)

---

## 1. 问题场景与设计动机

### 1.1 场景描述

本项目的机械臂（UR5）在中国象棋棋盘上方操作。棋盘区域约 0.54m × 0.60m，机械臂 base_link 位于棋盘外侧。棋子是直径 0.045m、高 0.018m 的圆柱，散布在 9×10 的网格上。此外还有预设障碍柱和人手安全区。

### 1.2 升级前的问题

升级前，轨迹规划采用**基于 safe-height 模板的动作分解 + 解析 IK 逐点求解 + 距离阈值反应式调速**：

```
每个 MotionPrimitive → 1 个 IK waypoint → 速度模式 ("fast"/"safe")
```

核心缺陷：

| 缺陷 | 表现 | 后果 |
|------|------|------|
| 无路径搜索 | 每个 primitive 只有 1 个 target waypoint | 无法绕过障碍物 |
| 无线段碰撞检测 | 只检查目标点的 2D 距离 | 线段可能穿过障碍物 |
| 无 waypoint 插值 | 关节空间跳变 | 控制不平滑，抖动大 |
| 无轨迹平滑 | 无 | jagged motion, 高 jerk |
| 遇到障碍只降速 | `_choose_speed_mode()` 返回 "safe" | 不改变路径，只减速 |

### 1.3 设计目标

在不改变 A/B/D 模块接口的前提下，增强 C 的 planning 模块：

1. 棋盘上方（z_safe 平面）能主动规划绕过障碍物的路径
2. 输出平滑、可执行的关节轨迹
3. 确定性输出（同一输入 → 同一输出），方便 demo 录屏
4. 向后兼容（可通过 `enable_*` 参数关闭新功能）

---

## 2. 系统架构总览

### 2.1 文件结构

```
src/planning/
├── collision_checker.py    # NEW: 路径线段 vs 障碍物碰撞检测
├── path_search.py          # NEW: A* 路径搜索 on 2D Grid
├── trajectory_smoother.py  # NEW: waypoint 插值 + 轨迹平滑
├── trajectory_planner.py   # MODIFY: 集成上述模块
├── obstacle_map.py         # MODIFY: 人手区使用 HORIZONTAL_CYLINDER
├── motion_primitives.py    # UNCHANGED
├── ik_solver.py            # UNCHANGED
└── chessboard_mapping.py   # UNCHANGED
```

### 2.2 升级后的数据流

```
MotionPrimitive[] → plan_trajectory()
  ├─ 对于水平移动 (approach/transfer):
  │    ├─ collision_checker.direct_path_clear() ?
  │    │    ├─ YES → 直接直线 + 插值
  │    │    └─ NO  → path_search.a_star_2d()
  │    │             → trajectory_smoother.shortcut_smoothing()
  │    │             → trajectory_smoother.interpolate_waypoints_cartesian()
  │    │             → IK per interpolated point
  │    └─ 对于垂直移动 (descend/lift/grasp/detach/retreat):
  │         → 插值直线 → IK per point
  └─ → trajectory_smoother.smooth_joint_trajectory()
       → JointTrajectory (with per-waypoint speed_profile)
```

### 2.3 为什么选择 2.5D 简化

棋盘操作本质是 **2.5D**：
- 水平转移发生在 `z_safe` 平面（0.18m 高度）
- 垂直下降/抬升发生在固定 (x, y) 列
- 所有障碍物（棋子、预设柱、人手区）都可以投影到 2D

这让我们可以将路径搜索从 3D 降维到 2D，大幅降低计算复杂度。

---

## 3. 碰撞检测

### 3.1 核心算法：线段 vs 障碍物采样检测

```
算法: check_segment_collision(start_xyz, end_xyz, obstacles, step_size, safety_margin)

1. 计算线段长度 L = |end - start|
2. n_steps = max(2, ceil(L / step_size) + 1)
3. for i in 0..n_steps-1:
     t = i / (n_steps - 1)
     sample_point = start + t * (end - start)
     for each obstacle:
         clearance = point_obstacle_clearance(sample_point, obstacle)
         if clearance <= safety_margin → COLLISION
4. 记录全局 min_clearance
5. 返回 CollisionCheckResult(collision_free, min_clearance, collision_point)
```

**步长选择：** `step_size = 0.005m`（5mm），在精度和性能之间平衡。0.3m 的线段约需 60 个采样点。

### 3.2 障碍物形状分派

```python
# 竖直圆柱（棋子、预设柱）
VERTICAL_CYLINDER:
    2D 投影 = 圆心 (cx, cy)，半径 r
    clearance = hypot(px - cx, py - cy) - r

# 水平圆柱（人手安全区）
HORIZONTAL_CYLINDER:
    2D 投影 = AABB 矩形
    - 沿 X 轴: [cx - height/2, cx + height/2]
    - 沿 Y 轴: [cy - radius, cy + radius]
    clearance = hypot(px - clamp(px, xmin, xmax), py - clamp(py, ymin, ymax))
```

**人手安全区 AABB 投影示意：**

```
旧方案 (Circle R=0.12):          新方案 (AABB ~0.24×0.05):

     ████████████                     
   ██            ██                   
  ██              ██                  
 ██                ██                 
██                  ██       ████████████████████
██                  ██       ████████████████████
 ██                ██                 
  ██              ██                  
   ██            ██                   
     ████████████                     

阻塞面积 ≈ 0.045 m²              阻塞面积 ≈ 0.012 m²
浪费 73% 可通行空间                 精确匹配实际手区
```

---

## 4. 路径搜索算法对比

### 4.1 A* (A-Star) — ✅ 本项目采用

**原理：**

A* 是 Dijkstra 算法的启发式增强版。它在图上搜索时使用评估函数：

```
f(n) = g(n) + h(n)

其中:
  g(n) = 从起点到节点 n 的实际代价
  h(n) = 从节点 n 到终点的启发式估计代价
```

**关键性质：**
- 当 h(n) ≤ 实际代价（admissible）时，A* 保证找到最优路径
- 当 h(n) 满足三角不等式（consistent）时，A* 每个节点只展开一次
- Euclidean distance 既是 admissible 也是 consistent

**本项目配置：**
- 搜索空间：2D 网格，分辨率 0.02m，约 45×45 = 2025 个格子
- 连接方式：8-邻域（水平/垂直/对角线）
- 对角线代价：√2 × 0.02m（真实几何距离）
- 启发式：Euclidean distance（admissible + consistent）
- 时间复杂度：O(N log N)，N = 网格数，实际 < 5ms

**伪代码：**
```
function A*(start, goal, grid):
    open_set = PriorityQueue()
    open_set.push(start, f_score=heuristic(start, goal))
    g_score[start] = 0
    
    while open_set not empty:
        current = open_set.pop()  # 最小 f_score
        
        if current == goal:
            return reconstruct_path(came_from, current)
        
        for each neighbor of current (8-connected):
            if neighbor is occupied or out of bounds:
                continue
            tentative_g = g_score[current] + move_cost * resolution
            
            if tentative_g < g_score[neighbor]:
                g_score[neighbor] = tentative_g
                f_score = tentative_g + heuristic(neighbor, goal)
                came_from[neighbor] = current
                open_set.push(neighbor, f_score)
    
    return FAILURE
```

### 4.2 RRT (Rapidly-exploring Random Tree)

**原理：**

RRT 通过随机采样在连续空间中增量式构建树。

```
算法: RRT(start, goal, obstacles, max_iter)

1. 初始化树 T = {start}
2. for i in 1..max_iter:
     q_rand = random_sample(workspace)
     q_near = nearest_neighbor(T, q_rand)
     q_new = steer(q_near, q_rand, step_size)
     
     if collision_free(q_near → q_new):
         T.add(q_new)
         T.add_edge(q_near, q_new)
         
         if distance(q_new, goal) < goal_tolerance:
             return extract_path(T, q_new)
3. return FAILURE
```

**RRT-Connect 变体：** 从起点和终点同时生长两棵树，交替扩展直到相遇。比基础 RRT 收敛更快。

**优点：**
- 不需要离散化空间
- 能处理高维空间（机械臂的 6D 关节空间）
- 概率完备（只要有解，时间足够长一定找到）

**缺点：**
- **随机性：** 每次运行产生不同路径，不可复现
- **路径不光滑：** 输出锯齿状路径，需要后处理
- **非最优：** 不能保证最短路径（RRT* 可以改进）

**为什么不选 RRT：**
- 我们的棋盘场景低维（2.5D）、结构化
- 需要确定性输出（demo 录屏可复现）
- 搜索空间小，A* 的离散化开销可忽略
- A* 保证最短路径，RRT 不保证

### 4.3 RRT* (RRT-Star) — 渐进最优变体

**改进点：**

RRT* 在 RRT 基础上增加了两个关键步骤：

```
1. ChooseParent: 新节点 q_new 在其邻域内搜索最优父节点
   → 使得到达 q_new 的路径代价最小

2. Rewire: q_new 成为父节点后，重新连接邻域内其他节点
   → 如果通过 q_new 到达它们的代价更小，就重连

理论保证:
  lim_{n→∞} P(RRT* 返回最优路径) = 1
```

**时间复杂度：** O(N log N) per iteration（含邻域搜索），比基础 RRT 慢。

**在本项目中的适用性：** 低。A* 在 2D 网格上直接返回最优路径，不需要渐进逼近。

### 4.4 PRM (Probabilistic Roadmap)

**原理：**

PRM 分为两个阶段：
1. **构建阶段：** 在自由空间中随机撒点，连接邻近点形成路图
2. **查询阶段：** 在路图上用 A* 或 Dijkstra 搜索

**为什么不选 PRM：**
- 构建阶段开销大（需要生成路图并验证碰撞）
- 场景动态变化（每次 pick/place 后棋盘占用变化），路图需重建
- 对于棋盘规模的场景，直接 A* 更高效

### 4.5 算法对比总结

| 方法 | 最优性 | 确定性 | 复杂度 | 高维适应 | 适合本场景 |
|------|--------|--------|--------|----------|------------|
| **A* on 2D Grid** | ✅ 最优 | ✅ 确定 | O(N log N) | ❌ | ⭐⭐⭐ |
| RRT | ❌ 不保证 | ❌ 随机 | O(N log N) | ✅ | ⭐⭐ |
| RRT* | ✅ 渐进最优 | ❌ 随机 | O(N log²N) | ✅ | ⭐⭐ |
| RRT-Connect | ❌ 不保证 | ❌ 随机 | O(N log N) | ✅ | ⭐⭐ |
| PRM | ✅ (在路图上) | ❌ 随机 | O(N²) build | ✅ | ⭐ |

---

## 5. A* on 2D Grid 实现细节

### 5.1 坐标映射

```
世界坐标 → 网格索引:
  gx = int((x - xmin) / resolution)
  gy = int((y - ymin) / resolution)

网格索引 → 世界坐标 (格子中心):
  x = xmin + (gx + 0.5) * resolution
  y = ymin + (gy + 0.5) * resolution
```

### 5.2 占栅格生成

```python
def build_2d_occupancy_grid(obstacles, z_plane, resolution, bounds):
    for each obstacle:
        if shape == VERTICAL_CYLINDER:
            # 膨胀半径 = obstacle.radius + safety_margin
            for each grid cell within inflated radius:
                if cell_center.distance_to(obstacle_center) <= inflated_radius:
                    grid[cell] = OCCUPIED
        
        elif shape == HORIZONTAL_CYLINDER:
            # 膨胀 AABB
            half_len_x = obstacle.height / 2 + safety_margin
            half_len_y = obstacle.radius + safety_margin
            for each grid cell within AABB:
                grid[cell] = OCCUPIED
```

### 5.3 边界与安全处理

```python
# 1. 网格边界 → 一律 OCCUPIED（隐式：不生成边界外的格子）
# 2. 起点/终点在障碍中 → BFS 搜索最近可通行格子
# 3. 搜索超时 → 返回 success=False（100ms timeout）
# 4. 确定性保证 → neighbor 遍历顺序固定
```

### 5.4 搜索空间

默认 bounds 基于机械臂工作空间（以 base_link 为中心，半径 0.25-0.9m 的可达圆环）：

```
xmin = base_x - 0.85, xmax = base_x + 0.85
ymin = base_y - 0.85, ymax = base_y + 0.85
```

网格数：约 85/0.02 × 85/0.02 ≈ 1800 个格子。

---

## 6. Waypoint 插值与轨迹平滑

### 6.1 为什么需要插值和平滑

A* 输出粗粒度路径点（相邻 ~0.02m），直接送 IK 会产生问题：

```
问题 1: 关节空间跳变
  A* 路径: (0.00, 0.00) → (0.02, 0.02) → (0.04, 0.02)
  IK 求解: θ_a → θ_b → θ_c
  问题: Cartesian 空间 2cm 的跳变在关节空间可能对应较大角度变化

问题 2: A* 锯齿
  8-邻域连接产生 0°/45°/90° 的限制方向，路径呈锯齿状
  示例: →↗→↘→ (jagged)

问题 3: 控制不平滑
  稀疏 waypoint → 控制器在两个 waypoint 间做线性插值 → 加速度不连续
```

### 6.2 三级平滑流水线

```
A* 原始路径 (粗粒度、锯齿)
  │
  ▼ Phase 1: Shortcut 平滑
  简化锯齿、保留碰撞安全
  │
  ▼ Phase 2: Cartesian 插值
  加密 waypoint 密度
  │
  ▼ Phase 3: 关节空间平滑
  移动平均去噪
  │
  ▼ 最终 JointTrajectory
```

### 6.3 Phase 1: Shortcut 平滑

**算法：贪心跳过冗余 waypoint**

```
输入: path = [p0, p1, p2, ..., pn]  (A* 输出)
输出: smoothed_path

smoothed = [p0]
i = 0
while i < len(path) - 1:
    # 从最远端回退尝试，找最长可行跳跃
    for j from len(path)-1 down to i+2:
        if direct_path_clear(path[i], path[j]):
            smoothed.append(path[j])
            i = j
            break  # 贪心：取最长跳跃
    else:
        # 无可行跳跃，前进一格
        i += 1
        smoothed.append(path[i])

return smoothed
```

**效果示意：**

```
A* 输出 (8-邻域锯齿):
  ●─●─●
        ●
          ●─●─●
              
Shortcut 后:
  ●─────────●─────●
```

**为什么用 Shortcut 而不用 B-spline：**
- Shortcut 每次跳跃都做碰撞检测 → **保证碰撞安全**
- B-spline 拟合可能让曲线偏离到障碍物内
- Shortcut 简单、可预测、易于调试

### 6.4 Phase 2: Cartesian 线性插值

**算法：均匀采样线段**

```
输入: path_xyz = [p0, p1, ..., pm]  (shortcut 后的 3D 点)
输出: 密集插值序列

for each adjacent pair (p_i, p_{i+1}):
    seg_length = |p_{i+1} - p_i|
    n_steps = max(1, ceil(seg_length / step_size))
    
    for j in 1..n_steps:
        t = j / n_steps
        result.append(p_i + t * (p_{i+1} - p_i))
```

**步长参数：**
- 水平移动：`waypoint_interpolation_step = 0.03m`
- 垂直移动：`waypoint_vertical_step = 0.01m`（垂直方向更精细，因为靠近棋子时需要更精确的控制）

### 6.5 Phase 3: 关节空间移动平均平滑

**算法：对每个关节维度独立做 1D 移动平均**

```
输入: joint_waypoints = [θ₀, θ₁, ..., θₙ]  (IK 求解后的 6D 关节角)
窗口: smoothing_window = 3 (奇数)

for each joint j in [0..5]:
    for each index k in [0..n-1]:
        left = max(0, k - half)
        right = min(n-1, k + half)
        smoothed[k][j] = mean(original[left:right+1][j])
```

**数学本质：** 1D 离散卷积，核 = [1/3, 1/3, 1/3]

**效果：** 减小相邻 waypoint 之间的关节角阶跃变化（jerk 的代理指标）。边界处自适应使用较小窗口。

**信号处理视角：**

移动平均等价于低通滤波。它衰减了高频分量（快速角度变化），保留了低频分量（整体运动趋势）：

```
原始信号:       平滑后:
  /\  /\           ───╲___╱───
 /  \/  \    →      (高频毛刺被滤除)
/        \

频率响应: H(ω) = sinc(ω * window/2)
截止频率: f_c ≈ 1 / (window * Δt)
```

### 6.6 为什么不是 Jerk-Optimal Trajectory

更高级的轨迹优化方案（如 minimum-jerk、minimum-snap、B-spline fitting）可以提供更好的平滑效果，但：

1. **计算开销大：** 需要解 QP (Quadratic Programming) 或 NLP (Nonlinear Programming)
2. **碰撞约束难处理：** 在优化问题中嵌入碰撞检测约束复杂
3. **实时性要求：** 本系统需要在 100ms 内完成规划
4. **收益有限：** 棋盘操作速度慢、精度要求中等，移动平均已足够

如果未来需要更精准的控制（如高速连续操作），可以考虑升级到 Jerk-optimal trajectory generation。

---

## 7. 人手安全区精确建模

### 7.1 问题

人手安全区在 PyBullet 仿真中是一个沿 X 轴的水平横躺圆柱：

```
Visual (B module):
  GEOM_CYLINDER
  radius = 0.025m (横截面半径)
  length = 0.24m (4 cells × 0.06m)
  orientation: rotated 90° around Y → cylinder axis along X
  center: z_safe plane
```

旧版 Planning (C module) 用一个竖直圆柱近似：
```
Obstacle(radius = 0.12 (length/2), height = 0.05)
→ 在 z_safe 平面投影为半径 0.12m 的大圆
```

这浪费了 73% 的可通行空间。

### 7.2 解决方案

新增 `ObstacleShape` 枚举和对应处理逻辑：

```python
class ObstacleShape(str, Enum):
    VERTICAL_CYLINDER = "vertical_cylinder"     # 现有棋子/预设柱
    HORIZONTAL_CYLINDER = "horizontal_cylinder" # 人手安全区
    AABB = "aabb"                                # 预留扩展

@dataclass(frozen=True)
class Obstacle:
    # ... 原有字段 ...
    shape: ObstacleShape = ObstacleShape.VERTICAL_CYLINDER  # 默认兼容
    orientation_rpy: tuple[float, float, float] = (0.0, 0.0, 0.0)
```

碰撞检测和占栅格生成根据 `shape` 分派：
- `VERTICAL_CYLINDER` → 圆形碰撞检测 / 圆形占栅格
- `HORIZONTAL_CYLINDER` → AABB 碰撞检测 / AABB 占栅格

---

## 8. 集成流程与数据流

### 8.1 plan_trajectory() 完整流程

```python
def plan_trajectory(primitives_or_contexts, obstacles, config, *, 
                    enable_path_search, enable_smoothing, enable_interpolation):
    
    cartesian_waypoints = []
    speed_profile = []
    
    for each primitive:
        obstacles = context.obstacles  # 逐帧变化的棋盘占用
        
        if primitive.type in ("approach", "transfer"):
            # === 水平移动 ===
            start_xy = last_waypoint.xy
            end_xy = primitive.target.xy
            
            if enable_path_search and NOT direct_path_clear(start_xy, end_xy):
                # 路径被阻挡 → A* 绕行
                result = a_star_2d(start_xy, end_xy, obstacles)
                
                if result.success:
                    path_3d = [(x, y, z_safe) for x, y in result.path_xy]
                    
                    if enable_smoothing:
                        path_3d = shortcut_smoothing(path_3d, obstacles)
                    
                    if enable_interpolation:
                        path_3d = interpolate_cartesian(path_3d)
                    
                    cartesian_waypoints.extend(path_3d[1:])  # skip first
                    speed_profile.extend(["safe"] * len(path_3d))
                else:
                    # A* 失败 → fallback 直接路径
                    cartesian_waypoints.append(primitive.target)
                    speed_profile.append("safe")
            else:
                # 直接路径无障碍 → 插值直线
                if enable_interpolation:
                    segment = interpolate([prev, primitive.target])
                    cartesian_waypoints.extend(segment[1:])
                else:
                    cartesian_waypoints.append(primitive.target)
                speed_profile.append("fast")
        else:
            # === 垂直移动 ===
            if enable_interpolation:
                segment = interpolate([prev, primitive.target], vertical_step)
                cartesian_waypoints.extend(segment[1:])
            else:
                cartesian_waypoints.append(primitive.target)
            speed_profile.append("safe")
    
    # IK 批量求解
    joint_waypoints = [solve_ik(wp) for wp in cartesian_waypoints]
    
    # 关节空间平滑
    if enable_smoothing:
        joint_waypoints = smooth_joint_trajectory(joint_waypoints)
    
    return JointTrajectory(joint_waypoints, speed_profile)
```

### 8.2 向后兼容性

三个 `enable_*` 参数默认 True：

```python
# 完全向后兼容：关闭所有新功能
plan_trajectory(primitives, obstacles, 
                enable_path_search=False,
                enable_smoothing=False, 
                enable_interpolation=False)
# → 行为与旧版完全一致：每个 primitive = 1 waypoint
```

### 8.3 A/B/D 模块影响

**零改动。** 所有变更都在 `src/planning/` 内部：
- A 仍输出 `MoveCommand → LogicalAction`
- B 仍接收 `RobotHandle/SceneHandle`
- D 仍接收 `JointTrajectory`（waypoint 变多但结构不变，D 本身支持任意长度）

---

## 9. 性能分析与参数调优

### 9.1 各阶段耗时估计

| 阶段 | 典型耗时 | 影响因素 |
|------|----------|----------|
| 碰撞检测 (per segment) | < 1ms | 障碍物数量、步长 |
| A* 搜索 | < 5ms | 网格分辨率、搜索空间 |
| Shortcut 平滑 | < 1ms | 路径点数 |
| Cartesian 插值 | < 1ms | 路径点数、步长 |
| IK 求解 (per waypoint) | ~0.3ms | 迭代次数 |
| 关节平滑 | < 1ms | waypoint 数 |

**总耗时：** 一个典型的 pick+place（8 个 primitive）约需 10-20ms，远低于 100ms 的实时要求。

### 9.2 关键参数

| 参数 | 默认值 | 作用 | 调优方向 |
|------|--------|------|----------|
| `path_grid_resolution` | 0.02m | A* 网格精度 | 减小→更精细路径，增大→更快搜索 |
| `path_collision_check_step` | 0.005m | 碰撞检测步长 | 减小→更安全，增大→更快 |
| `path_search_timeout_ms` | 100ms | A* 超时 | 增大→允许更复杂绕行 |
| `waypoint_interpolation_step` | 0.03m | 水平插值密度 | 减小→更平滑，增大→更快 |
| `waypoint_vertical_step` | 0.01m | 垂直插值密度 | 同水平 |
| `safety_margin` | 0.015m | 障碍物膨胀 | 增大→更保守安全 |
| `smoothing_window` | 3 | 关节平滑窗口 | 增大→更平滑但可能偏离目标 |

### 9.3 典型场景性能

```
场景: A1 → A2 (简单移动，无障碍)
  primitives: 8
  旧版 waypoints: 8
  新版 waypoints (含插值): ~55-60
  规划耗时: ~10ms
  执行耗时: ~3s

场景: A1 → E5 (跨棋盘，有障碍物阻挡)
  primitives: 8
  新版 waypoints: ~100-120 (含绕行路径)
  规划耗时: ~15ms
  A* 展开节点数: ~500-1000
```

---

## 10. 扩展方向

### 10.1 从 A* 升级到 Theta*

**动机：** 8-邻域 A* 的路径受限于 45° 增量方向，即使经过 shortcut 平滑仍不够自然。

**Theta* 改进：** 在 A* 搜索时允许任意角度的父子连接（Line-of-Sight），直接产生更短的路径。

```
A* + Shortcut:        Theta*:
  ●─●─●                 ●
       ●                 ╲
         ●─●               ●
                            ╲
                              ●
```

**实现难度：** 中等。需要在 A* 框架中加入 Line-of-Sight 检查。

### 10.2 3D 路径搜索

当前 2.5D 简化假设所有运动在同一平面。如果未来需要在不同高度之间做绕行（如障碍物高于 z_safe），需要升级到 3D A* 或 3D RRT。

### 10.3 Jerk-Optimal / Minimum-Snap 轨迹

当前移动平均平滑是启发式方法。如果需要更优的轨迹质量，可以：

1. 对关节空间 waypoint 做 cubic/quadratic spline 拟合
2. 用 QP solver 做 minimum-jerk trajectory generation
3. 用 TOPP (Time-Optimal Path Parameterization) 做时间优化

### 10.4 学习型路径规划

利用棋盘的结构化特性，可以预计算常用路径模板（如每条列之间的 transfer path），运行时直接查表，避免重复搜索。

---

## 参考文献

1. Hart, P. E., Nilsson, N. J., & Raphael, B. (1968). "A Formal Basis for the Heuristic Determination of Minimum Cost Paths." *IEEE Transactions on Systems Science and Cybernetics*.
2. LaValle, S. M. (1998). "Rapidly-exploring Random Trees: A New Tool for Path Planning." *TR 98-11, Computer Science Dept., Iowa State University*.
3. Karaman, S. & Frazzoli, E. (2011). "Sampling-based Algorithms for Optimal Motion Planning." *IJRR*.
4. Kavraki, L. E., et al. (1996). "Probabilistic Roadmaps for Path Planning in High-Dimensional Configuration Spaces." *IEEE T-RA*.
5. Nash, A., et al. (2007). "Theta*: Any-Angle Path Planning on Grids." *AAAI*.
6. Richter, C., Bry, A., & Roy, N. (2016). "Polynomial Trajectory Planning for Aggressive Quadrotor Flight in Dense Indoor Environments." *ISRR*.
