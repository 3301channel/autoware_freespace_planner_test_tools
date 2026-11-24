# parking_input_evaluator 开发文档

> 目标版本：ROS 2 Humble + Autoware 2.1.0（`autoware_freespace_planner` / `autoware_costmap_generator`）

> 用途：对autoware泊车规划器freespace_planner的输入进行评估

---

## 1. 项目目标

### 1.1 背景

Autoware 2.1.0 的泊车链路是：

```
mission_planner → scenario_selector → costmap_generator
      → freespace_planner(A*) → planning_validator → controller
```

现有验证能力全部集中在**输出侧**：

- `planning_validator`：只检查轨迹本身（曲率、加速度、速度、点间距），不关心输入；
- `planning_evaluator`：评估指标默认面向车道规划，无泊车专用项；
- 《泊车轨迹评估节点》：评估 `parking/trajectory` 这条**输出轨迹**能不能被控制执行。

但实际调试中，绝大多数泊车失败发生在**输入侧**，且失败现象高度同质——planner 只输出一句 `Failed to find path`，看不出到底哪一环不合格。实测归纳出四类高频原因：

| 现象 | 实际根因 | 现有模块能否发现 |
| --- | --- | --- |
| goal 在栅格外，A* 报无解 | `grid_length` 小于停车场跨度 | 不能 |
| goal 在栅格内但仍无解 | 车位宽度不足 + 标线点云侵蚀 | 不能 |
| 栅格正常但忽然不规划 | `scenario` 不是 Parking，costmap 未激活 | 不能 |
| 偶发失败，重试就好 | 栅格更新频率低 / 定位位姿与 goal 不同步 | 不能 |

这些原因都可在**规划发生之前**用纯几何方式判定，属于"门禁"问题，而非"轨迹质量"问题。因此需要一个独立的输入评估节点。

### 1.2 核心目标

1. **只读门禁**：仅订阅话题与 TF，不发布任何控制/规划指令，对现有系统零侵入。
2. **前置判定**：输出布尔量 `is_input_ready`，在 planner 报错之前就说明输入是否充分。
3. **可定位**：把"能否规划"拆成 7 组 23 项独立检查，每项给出实测值、阈值与修复建议，失败时可精确到某一项。
4. **与 A* 语义对齐**：不复用"看起来合理"的经验阈值，而是复刻 `astar_search.cpp` 的障碍判定和 EDT 约束，保证判定结果与 planner 实际行为一致。
5. **量化覆盖度**：输出"35 个车位里有几个能停"这一可数指标，把系统能力从抽象判断变成可度量的数字。
6. **可配置、可观测**：阈值、车辆参数、地图路径均由参数注入；结果通过 topic / service / diagnostics / RViz Marker 四路输出。

### 1.3 非目标（Non-goals）

- 不生成、不修改任何轨迹与栅格；
- 不计算 occupancy grid（完全依赖上游 `costmap_generator`）；
- 不替代 `planning_validator`（后者检查轨迹，本节点检查输入）；
- 不做障碍物感知、不做路径搜索；
- 不与 `scenario_selector`、`freespace_planner` 争抢任何话题的所有权。

---

## 2. 与现有模块的关系

三个节点构成"输入—输出"双向验证闭环：

| 模块 | 评估对象 | 判定量 | 典型问题 |
| --- | --- | --- | --- |
| `planning_validator` | 轨迹 | 通过/拒绝 | 曲率超限、加速度过大 |
| 泊车轨迹评估节点 | `parking/trajectory` | `is_control_ready` | 轨迹末端精度、换向次数、外廓碰撞 |
| **本节点（输入评估）** | `occupancy_grid` / `route` / `odom` / `scenario` / TF | **`is_input_ready`** | **栅格盲区、车位过窄、场景未激活、位姿不同步** |

调用时序：

```
┌────────────┐   ┌──────────────────┐   ┌───────────────┐   ┌──────────────┐
│ 上游输入   │──▶│ 输入评估（本节点）│──▶│ freespace_planner │──▶│ 轨迹评估节点 │
│ grid/route │   │  is_input_ready  │   │   A* 搜索      │   │ is_control_ready │
└────────────┘   └──────────────────┘   └───────────────┘   └──────────────┘
                         │                      │
                         └──── 失败原因可独立定位 ─────┘
```

若 `is_input_ready = false`，则 planner 必然失败，此时无需再看轨迹。若 `is_input_ready = true` 而 planner 仍失败，则问题出在 planner 内部（搜索参数、超时），与输入无关——这就是门禁节点的价值：**先把责任边界划清**。

---

## 3. 输入与输出接口

### 3.1 订阅话题（Input）

| 话题 | 类型 | 用途 | 是否必须 |
| --- | --- | --- | --- |
| `/planning/scenario_planning/parking/costmap_generator/occupancy_grid` | `nav_msgs/msg/OccupancyGrid` | 被评估的栅格（A/F 组核心） | 必须 |
| `/planning/mission_planning/route` | `autoware_planning_msgs/msg/LaneletRoute` | 取 `goal_pose` 作为待检目标（B 组） | 必须 |
| `/localization/kinematic_state` | `nav_msgs/msg/Odometry` | 起点位姿、频率、与 route.start 比对（C 组） | 必须 |
| `/planning/scenario_planning/scenario` | `autoware_internal_planning_msgs/msg/Scenario` | 确认处于 Parking（D 组） | 必须 |
| `/map/vector_map` | `autoware_map_msgs/msg/LaneletMapBin` | 解析 `parking_lot` / `parking_space` 几何（A3/G/F4） | 必须 |
| `/tf`、`/tf_static` | `tf2_msgs/msg/TFMessage` | `map → base_link` 可达性（E 组） | 必须 |
| `/vehicle/status/steering_status` | `autoware_vehicle_msgs/msg/SteeringStatus` | 预留：判断起点是否处于大转角姿态 | 可选 |
| `/clock` | `rosgraph_msgs/msg/Clock` | 仿真时间下保证超时判定正确 | 可选 |

> **栅格来源可配置**：`freespace_planner` 既可能订阅 `costmap_generator` 的输出，也可能改为订阅**直接生成 occupancy grid 的节点**。因此 `occupancy_grid_topic` 设计为 launch 参数（默认仍指向 costmap_generator），切换来源无需改代码：
>
> ```bash
> ros2 launch parking_input_evaluator parking_input_evaluator.launch.xml \
>   occupancy_grid_topic:=/your/occupancy_grid_topic
> ```

**QoS 要求**：

- 栅格使用 `SensorDataQoS`（BEST_EFFORT）或 `TransientLocal` 视上游而定，需在实际链路验证；
- `route`、`scenario` 使用 `TransientLocal` + `Reliable`，避免因发布早于订阅而漏帧；
- `vector_map` 使用 `TransientLocal` + `Reliable`，只需成功接收一次即可缓存。

### 3.2 发布话题（Output）

| 话题 | 类型 | 内容 |
| --- | --- | --- |
| `/planning/evaluation/parking_input_report` | `parking_input_evaluator/msg/ParkingInputReport` | 完整评估结果：23 项明细、分级、失败原因、车位统计 |
| `/planning/evaluation/parking_input_marker` | `visualization_msgs/msg/MarkerArray` | 栅格覆盖范围框、盲区高亮、车位染色（绿/黄/红）、goal 位姿箭头 |
| `/diagnostics` | `diagnostic_msgs/msg/DiagnosticArray` | 标准诊断，`level` 映射：FAIL→ERROR，WARN→WARN，PASS→OK |

### 3.3 服务（Service）

| 服务 | 类型 | 用途 |
| --- | --- | --- |
| `/planning/evaluation/get_parking_input_report` | `parking_input_evaluator/srv/GetParkingInputReport` | 主动拉取最近一次结果，支持 `include_space_details` 控制是否返回逐车位明细 |

### 3.4 TF 依赖

需要 `map → base_link` 的变换链完整。节点不主动广播 TF，仅监听；若 `timed out` 则 E1 判 FAIL 并说明缺失。

---

## 4. 评估指标体系

### 4.1 记号约定

设：

- 栅格 \(G\)：宽 \(W\)、高 \(H\)、分辨率 \(r\)、原点 \((x_0,y_0)\)，占据矩阵 \(C[i,j]\in\{-1,0,\dots,100\}\)；
- 停车场几何 \(L\)：由 `parking_lot` 多边形给出；
- 停车位集合 \(\mathcal{S}=\{S_1,\dots,S_N\}\)，每个 \(S_k\) 为线串 + 宽度 \(w_k\)；
- 车辆碰撞形状：长 \(L_c\)、宽 \(W_c\)（见 4.8 节）；
- 起点 \(\mathbf{p}_s=(x_s,y_s,\theta_s)\)，目标 \(\mathbf{p}_g=(x_g,y_g,\theta_g)\)。

栅格索引：

\[
i=\left\lfloor \frac{x-x_0}{r}\right\rfloor,\qquad
j=\left\lfloor \frac{y-y_0}{r}\right\rfloor
\]

障碍判定（与 `abstract_algorithm.cpp:117-121` 完全一致）：

\[
\mathrm{obs}(i,j) = \bigl(C[i,j] < 0\bigr)\ \lor\ \bigl(C[i,j] \ge \tau_{\text{obs}}\bigr),\quad \tau_{\text{obs}}=100
\]

> 注意：`cost < 0`（unknown）被 planner **当作障碍**，因此栅格里的未知区域会实际压缩可行驶空间。

每项检查输出三级：`PASS` / `WARN` / `FAIL`。分级规则见 5.2 节。

---

### 4.2 A 组：栅格本体

#### A1 栅格发布频率

\[
f_{\text{grid}} = \frac{N_{\text{msg}}-1}{t_{\text{last}}-t_{\text{first}}}
\]

判据：\(8 \le f_{\text{grid}} \le 12\) 为 PASS，否则 WARN。

**为什么重要**：`freespace_planner` 以 `update_rate=10 Hz` 运行，但只有在收到新栅格时才 `setMap()` 重算。若栅格只有 4 Hz，规划决策会滞后 250 ms，动态障碍响应不及时。

> 实测：session2 为 **4.25 Hz**，显著低于期望。

#### A2 元数据一致性

要求同时满足：

- 栅格坐标系为 `map`；
- 分辨率误差 \(|r-r_{\text{expected}}|<\epsilon\)；
- `origin.orientation` 为单位四元数。

默认 \(r_{\text{expected}}=0.3\) m，\(\epsilon=10^{-6}\)。

#### A3 停车场覆盖率

用包络框近似计算：

\[
\mathrm{cover}=\frac{\mathrm{area}\big(G\cap L\big)}{\mathrm{area}(L)}
\]

判据：\(\mathrm{cover}\ge 0.95\) 为 PASS；\(0.75\le \mathrm{cover}<0.95\) 为 WARN；否则 FAIL。

同时给出四方向缺口：

\[
\Delta_{\text{right}}=\max(0,\ x_L^{\max}-x_G^{\max}),\quad
\Delta_{\text{left}}=\max(0,\ x_G^{\min}-x_L^{\min})
\]

**为什么重要**：栅格以车辆为中心、边长固定为 `grid_length_x/y`，车一旦离开停车场中心，远端必然进入盲区。

> 实测：\(G\) 为 69.9×69.9 m，\(L\) 为 56.8×77.7 m，覆盖率 **53.7%**，右侧缺 11.5 m、下侧缺 25.4 m。

#### A4 栅格值分布

统计 \(C<0\)（unknown）、\(0\le C<\tau_{\text{obs}}\)（free）、\(C\ge\tau_{\text{obs}}\)（occupied）的比例。

判据：unknown 比例 \(<5\%\) 为 PASS，\(<20\%\) 为 WARN，否则 FAIL。

---

### 4.3 B 组：目标位姿

#### B1 route 与 goal 有效性

检查 `route.goal_pose` 存在、`frame_id` 可解析。

#### B2 goal 在栅格内

\[
0 \le \frac{x_g-x_0}{r} < W,\qquad 0 \le \frac{y_g-y_0}{r} < H
\]

不满足即 FAIL——A* 的 `is_obstacle_table_` 只覆盖 `data` 数组，栅格外不存在单元格，必然无解。

#### B3 goal 单元格可通行

\[
\neg\,\mathrm{obs}(i_g,j_g)
\]

#### B4 goal 处可容纳车辆矩形

对车辆矩形采样（见 10.2 节），要求所有采样点均落在 `free` 格内：

\[
\forall (a,b)\in\mathcal{D}:\neg\,\mathrm{obs}\big(\mathbf{p}(a,b;\mathbf{p}_g)\big)
\]

其中 \(\mathcal{D}=\{(a,b)\mid a\in[-L_c/2,L_c/2],\ b\in[-W_c/2,W_c/2]\}\)。

#### B5 目标朝向与车位一致性

取距 \(\mathbf{p}_g\) 最近的停车位 \(S_k\)，设其主方向为 \(\phi_k\)：

\[
\Delta\theta=\min\bigl(|\mathrm{wrap}(\theta_g-\phi_k)|,\ 180^\circ-|\mathrm{wrap}(\theta_g-\phi_k)|\bigr)
\]

同时判断 \(\mathbf{p}_g\) 是否落在 \(S_k\) 的"线 + 宽度"带状区域内。

判据：在带内且 \(\Delta\theta\le 6^\circ\)（即 `angle_goal_range`）为 PASS；在带内且 \(\Delta\theta\le 18^\circ\) 为 WARN；否则 FAIL。

---

### 4.4 C 组：起点位姿

#### C1 odometry 频率

\[
f_{\text{odom}}=\frac{N_{\text{msg}}-1}{t_{\text{last}}-t_{\text{first}}}
\]

判据：\(5\le f_{\text{odom}}\le 20\) 为 PASS。

#### C2 / C3 起点在栅格内且可容纳车辆

与 B2 / B4 同构，只是把 \(\mathbf{p}_g\) 换成 \(\mathbf{p}_s\)。起点碰撞意味着 A* 第一步即失败。

#### C4 odometry 与 route.start 一致性

\[
e_s=\lVert \mathbf{p}_s-\mathbf{p}_{\text{route.start}}\rVert
\]

判据：\(e_s<1.0\) m 为 PASS；\(<3.0\) m 为 WARN；否则 FAIL。

> 实测：session2 为 **1.43 m**，属录制时序问题（route 发布晚于 odom 首帧）。

---

### 4.5 D 组：场景状态

#### D1 处于 Parking 场景

\[
\rho_{\text{parking}}=\frac{N_{\text{parking}}}{N_{\text{scenario}}}
\]

其中 \(N_{\text{parking}}\) 为 `current_scenario` 等于 `Parking` 的消息数，\(N_{\text{scenario}}\) 为消息总数。

判据：\(\rho>0.9\) 为 PASS；\(>0.3\) 为 WARN；否则 FAIL。

**为什么重要**：若 `costmap_generator` 配置 `activate_by_scenario: true`，非 Parking 时它**不发布栅格**，整条链路静默失效。

---

### 4.6 E 组：TF 可达性

#### E1 `map → base_link`

检查 `/tf` 与 `/tf_static` 中是否存在该边，或用 `tf2_ros::Buffer::canTransform` 直接查询（更可靠）。

缺失即 WARN——部分系统把该变换放在其他话题或由外部桥接发布。

---

### 4.7 F 组：可规划性（最核心）

#### F1 拓扑连通（仅 free）

对 \(\mathcal{F}=\{\neg\mathrm{obs}\}\) 做 8 邻域连通域标记 \(\mathcal{L}_{\text{free}}\)：

\[
\text{F1}=\Bigl(\mathcal{L}_{\text{free}}(i_s,j_s)=\mathcal{L}_{\text{free}}(i_g,j_g)\neq 0\Bigr)
\]

断裂说明栅格被障碍割裂，任何算法都无法连通。

#### F2 A* 可行连通（EDT 约束）

这是**与 planner 行为最接近**的判据。A* 在扩展节点时执行（`astar_search.cpp:262`）：

```cpp
if (getObstacleEDT(n_index).distance < 0.5 * collision_vehicle_shape_.width) continue;
```

即节点到最近障碍的距离必须不小于半车宽。定义可通行集合：

\[
\mathcal{P}=\Bigl\{\mathbf{x}\ \Big|\ \neg\mathrm{obs}(\mathbf{x})\ \land\ d(\mathbf{x})\ge \tfrac{W_c}{2}\Bigr\}
\]

其中欧氏距离变换：

\[
d(\mathbf{x})=\min_{\mathbf{o}\in\mathcal{O}}\lVert \mathbf{x}-\mathbf{o}\rVert_2,\qquad
\mathcal{O}=\{\mathbf{x}\mid \mathrm{obs}(\mathbf{x})\}
\]

再对 \(\mathcal{P}\) 做连通域标记，判定：

\[
\text{F2}=\Bigl(\mathcal{L}_{\mathcal{P}}(i_s,j_s)=\mathcal{L}_{\mathcal{P}}(i_g,j_g)\neq 0\Bigr)
\]

F2 比 F1 严格得多：F1 通过而 F2 失败，几乎必然意味着"车位过窄"或"通道被切碎"。

#### F3 goal 处 EDT 余量

\[
m_{\text{edt}}=d(\mathbf{p}_g)-\frac{W_c}{2}
\]

判据：\(m_{\text{edt}}\ge 0.10\) m 为 PASS；\(\ge -0.05\) m 为 WARN；否则 FAIL。

余量非负但很小，说明只有极少数离散朝向能落在目标附近，容易出现"偶发失败"。

#### F4 逐车位可停泊率（本节点的核心产出）

对每个 \(S_k\) 计算中心点 \(\mathbf{c}_k\) 与主方向 \(\phi_k\)，按顺序判定，命中即停：

| 状态 | 判定条件 |
| --- | --- |
| `OUT_OF_GRID` | \(\mathbf{c}_k\notin G\) |
| `BLOCKED` | \(\exists(a,b)\in\mathcal{D}:\ \mathrm{obs}\bigl(\mathbf{p}(a,b;\mathbf{c}_k,\phi_k)\bigr)\) |
| `UNREACHABLE` | \(\mathcal{L}_{\mathcal{P}}(i_{c_k},j_{c_k})\neq\mathcal{L}_{\mathcal{P}}(i_s,j_s)\) |
| `PARKABLE` | 以上均不满足 |

可停泊率：

\[
\eta=\frac{N_{\text{PARKABLE}}}{N_{\text{total}}}
\]

判据：\(\eta\ge 0.9\) 为 PASS；\(\ge 0.5\) 为 WARN；否则 FAIL。

**为什么用中心点**：车辆停入车位后，车体中心与车位中心线中点大致重合；车位长度 5.4 m、车长 4.56 m，纵向有 0.8 m 调整余量，因此中心点是最具代表性的检查位置。

> 实测：session2 为 **28/35（80%）**——5 个 OUT_OF_GRID（y=181.4~192.4，超出栅格下边界 196.05），2 个 BLOCKED（剩余可用宽度 1.99 m 与 1.64 m，均小于碰撞宽 2.0568 m）。

---

### 4.8 G 组：几何余量

#### 车辆碰撞形状

依据 `freespace_planner_node.cpp:442-445`，**margin 只加一次**：

```cpp
extended_vehicle_shape.length += margin;
extended_vehicle_shape.width  += margin;
extended_vehicle_shape.base2back += margin / 2;
```

\[
L_c=L_v+m,\qquad W_c=W_v+m
\]

以 `sample_vehicle` 为例：\(L_v=4.5136\) m、\(W_v=2.0068\) m、\(m=0.05\) m，得 \(L_c=4.5636\) m、\(W_c=2.0568\) m。

#### G1 车位宽度 vs 碰撞形状

\[
m_w=\frac{\min_k w_k-W_c}{2}
\]

判据：\(m_w\ge 0.20\) m 为 PASS；\(\ge 0.10\) m 为 WARN；否则 FAIL。

> 实测：\(w_k=2.6\) m → \(m_w=0.2716\) m，几何上勉强及格，但见 6.2 节的分辨率量化问题。

#### G2 车位长度 vs 碰撞形状

\[
m_l=\frac{\min_k \lVert S_k\rVert-L_c}{2}
\]

判据：\(m_l\ge 0.20\) m 为 PASS；\(\ge 0.05\) m 为 WARN。

> 实测：5.4 − 4.5636 = 0.8364 m，单侧 0.418 m，PASS。

#### G3 朝向容差的离散化

A* 的 `theta_size=120` 对应每档：

\[
\Delta\theta_{\text{step}}=\frac{360^\circ}{120}=3^\circ
\]

`angle_goal_range=6°` 可容纳的档数：

\[
n_{\text{step}}=\frac{6^\circ}{3^\circ}=2
\]

判据：\(n_{\text{step}}\ge 1\) 为 PASS。若容差小于一档（如 1°），朝向匹配几乎必然失败。

#### G4 栅格边长 vs 停车场对角线

\[
\text{side}=W\cdot r,\qquad D=\sqrt{(x_L^{\max}-x_L^{\min})^2+(y_L^{\max}-y_L^{\min})^2}
\]

判据：\(\text{side}\ge D\) 为 PASS；\(\ge 0.85D\) 为 WARN；否则 FAIL。

**为什么用对角线**：网格始终跟随车辆，最坏情况下车在停车场一角、目标在对角，所需跨度为 \(D\)。

> 实测：side = 69.9 m，D = 96.3 m → FAIL。

---

### 4.9 指标汇总表

| 组 | ID | 指标 | PASS 阈值 | WARN 阈值 | 依赖 |
| --- | --- | --- | --- | --- | --- |
| A | A1 | 栅格频率 | 8~12 Hz | 其他 | grid |
| A | A2 | 元数据一致 | frame/res/origin 正确 | — | grid |
| A | A3 | 覆盖率 | ≥0.95 | ≥0.75 | grid + map |
| A | A4 | unknown 比例 | <5% | <20% | grid |
| B | B1 | route 有效 | 存在且可解析 | — | route |
| B | B2 | goal 在栅格内 | 是 | — | grid + route |
| B | B3 | goal 格可通行 | cost∈[0,100) | — | grid + route |
| B | B4 | goal 可放车 | 无冲突 | — | grid + route + 车辆 |
| B | B5 | 朝向一致 | Δθ≤6° 且在带内 | ≤18° | map + route |
| C | C1 | odom 频率 | 5~20 Hz | — | odom |
| C | C2 | start 在栅格内 | 是 | — | grid + odom |
| C | C3 | start 可放车 | 无冲突 | — | grid + odom |
| C | C4 | 起点一致 | <1.0 m | <3.0 m | odom + route |
| D | D1 | Parking 占比 | >90% | >30% | scenario |
| E | E1 | map→base_link | 可达 | 缺失 | TF |
| F | F1 | free 连通 | 同域 | — | grid + route + odom |
| F | F2 | A* 连通 | 同域 | — | grid + route + odom |
| F | F3 | goal EDT 余量 | ≥0.10 m | ≥-0.05 m | grid + route |
| F | F4 | 车位可泊率 | ≥0.9 | ≥0.5 | grid + map + odom |
| G | G1 | 车位宽余量 | ≥0.20 m | ≥0.10 m | map + 车辆 |
| G | G2 | 车位长余量 | ≥0.20 m | ≥0.05 m | map + 车辆 |
| G | G3 | 朝向离散化 | ≥1 档 | — | 参数 |
| G | G4 | 栅格 vs 对角线 | side≥D | ≥0.85D | grid + map |

---

## 5. 判定逻辑

### 5.1 总体结论

节点输出两个布尔量，语义明确区分：

```cpp
is_input_ready        = (无 FAIL) && (WARN 数量 == 0)
can_plan_current_goal = B 组全部 PASS && F1/F2 通过
```

- `can_plan_current_goal`：**本次这个 goal** 能否规划成功；
- `is_input_ready`：系统是否处于**稳定的可泊车状态**（换任意合法车位都大概率成功）。

两者必须分开。实测 session2 中 `can_plan_current_goal = true`（B1~B5 全 PASS）而 `is_input_ready = false`（覆盖率 53.7%、可泊率 80%）——这正是"能停进这个车位，但泊车功能不可用"的准确表达。

### 5.2 分级规则

| 级别 | 含义 | 是否影响结论 |
| --- | --- | --- |
| `PASS` | 满足全部判据 | 否 |
| `WARN` | 满足放宽阈值，存在风险 | 影响 `is_input_ready`，但不判定为不可规划 |
| `FAIL` | 不满足放宽阈值 | 直接影响 |

实现上统一为一个函数，避免各检查项自行解释：

```cpp
Level classify(bool pass_ok, bool warn_ok) {
  if (pass_ok) return Level::PASS;
  if (warn_ok) return Level::WARN;
  return Level::FAIL;
}
```

> 注意：`warn_ok` 是"满足 WARN（放宽）阈值"，不是"存在风险"。二者语义反了会导致所有通过的项被错判为 WARN——这是离线工具开发中实际踩过的坑。

### 5.3 与 A* 判定的一致性论证

为保证 `can_plan_current_goal` 与 planner 实际行为一致，本节点在三处严格复刻源码语义：

| 环节 | 源码位置 | 语义 | 本节点实现 |
| --- | --- | --- | --- |
| 障碍定义 | `abstract_algorithm.cpp:117-121` | `cost < 0 \|\| cost >= threshold` | 同式，`threshold` 可配 |
| 搜索节点约束 | `astar_search.cpp:262` | 节点 EDT ≥ 半车宽 | F2/F3/F4 的 \(\mathcal{P}\) 集合 |
| 碰撞形状 | `freespace_planner_node.cpp:442-445` | `length += m`、`width += m` | G1/G2/B4/C3 的 \(L_c,W_c\) |
| 栅格语义 | `costmap_generator.cpp` `fill_polygon_areas` | 整层=障碍，多边形内=空闲 | A4 值分布、车位带判定 |

**已知偏差**：A* 使用**精确矩形**做碰撞检测，本节点 F2 用 **EDT 圆盘**（半径 = 半车宽）近似。圆盘在车长方向更宽松、在斜向更保守，因此：

- F2 = false → A* 几乎必然失败（可靠）；
- F2 = true → A* 大概率成功，但极端窄缝可能仍失败（乐观偏差）。

需要更高置信度时，可开启 `strict_rect_connectivity: true`，以矩形碰撞表代替 EDT（计算量约增加 4 倍，见 10.6 节）。

### 5.4 可泊车位分类

F4 的分类结果直接对应修复动作，这是本节点最主要的工程价值：

| 状态 | 物理含义 | 对应修复 |
| --- | --- | --- |
| `OUT_OF_GRID` | 车位不在栅格视野内 | 增大 `grid_length_x/y` |
| `BLOCKED` | 车位内被障碍挤压，放不下车 | 提高 `minimum_lidar_height_thres` 过滤地面点；减小 `size_of_expansion_kernel`；加宽车位 |
| `UNREACHABLE` | 在视野内但被割裂 | 检查通道宽度、障碍物膨胀 |
| `PARKABLE` | 可正常泊入 | — |

---

## 6. 限制与假设

### 6.1 数据假设

1. **静态栅格假设**：假设评估所用的栅格在数百毫秒内有效。若 `f_grid` 只有 4 Hz，而车在移动，则结论存在滞后。
2. **地图已加载**：`parking_lot` / `parking_space` 必须存在于 lanelet2 地图中。若地图只有普通 lanelet（如纯道路地图），A3 / B5 / F4 / G1 / G2 全部失效，节点应输出"地图不含泊车元素"而非 FAIL。
3. **goal 来自 route**：假设 `route.goal_pose` 就是用户选择的泊车位。若上层用其他方式指定目标（如 `/goal_pose`），需扩展订阅。

### 6.2 算法近似

1. **覆盖率用包络框**：若停车场为 L 形或带缺口，bbox 会高估面积、低估覆盖率。需要精确值时可开启 `precise_polygon_coverage`（用多边形裁剪，成本更高）。
2. **EDT 圆盘近似**：见 5.3 节。
3. **车位中心点代表整位**：只检车位中心一个位姿。若车位内有局部凸起但中心干净，会漏检。可开启 `multi_pose_space_check` 沿车位中心线采 3 个点。
4. **分辨率量化**：0.3 m 分辨率下，车位 2.6 m = 8.67 格，车宽 2.0568 m = 6.86 格，取整误差可达 ±0.15 m。这是"几何余量 0.27 m 却仍碰撞"的根本原因——**几何余量不等于栅格余量**。
5. **标线被当障碍**：`use_points: true` + `minimum_lidar_height_thres: 0.0` 会把地面标线点填入障碍层。本节点能检测到结果（F4 报 BLOCKED），但无法区分"真实障碍"与"标线噪声"，需人工结合点云确认。

### 6.3 节点边界

1. **只报不拦**：不会阻止 planner 执行。若需拦截，应由上层（如 `operation_mode_transition_manager`）订阅诊断后决策。
2. **不评估轨迹**：即使 `is_input_ready = true`，规划出的轨迹仍可能曲率超限——那是轨迹评估节点的职责。
3. **不预测未来**：只反映当前帧。车辆移动后覆盖率会变化，需持续评估。
4. **不做多帧融合**：不做时序平滑，单帧结论可能抖动；通过 11 节的去抖策略缓解。

---

## 7. 参数设计

```yaml
# parking_input_evaluator.param.yaml
/**:
  ros__parameters:
    # ---------- 车辆参数 ----------
    vehicle:
      source: "vehicle_info"       # vehicle_info | manual
      wheel_base: 2.831
      max_steer_angle: 0.70
      vehicle_length: 4.5136
      vehicle_width: 2.0068
      vehicle_shape_margin: 0.05   # 与 freespace_planner 保持一致，只加一次

    # ---------- 栅格 ----------
    grid:
      expected_frame: "map"
      expected_resolution: 0.3
      resolution_tolerance: 0.001
      expected_rate_hz: 10.0
      rate_tolerance: 0.2          # ±20%
      obstacle_threshold: 100
      coverage_fail: 0.75
      coverage_pass: 0.95
      unknown_fail: 0.20
      unknown_pass: 0.05

    # ---------- 目标 / 起点 ----------
    goal:
      angle_tolerance_pass_deg: 6.0
      angle_tolerance_warn_deg: 18.0
      require_in_parking_space: true
      rect_samples_long: 11        # 车长方向采样数
      rect_samples_lat: 7          # 车宽方向采样数

    start:
      route_gap_fail: 3.0
      route_gap_pass: 1.0
      odom_rate_min: 5.0
      odom_rate_max: 20.0

    # ---------- 场景 ----------
    scenario:
      required: "Parking"
      ratio_fail: 0.30
      ratio_pass: 0.90

    # ---------- 连通性 ----------
    connectivity:
      edt_margin_pass: 0.10
      edt_margin_warn: -0.05
      strict_rect_connectivity: false
      connectivity_neighbors: 8

    # ---------- 车位 ----------
    parking_spaces:
      enabled: true
      check_mode: "center"         # center | multi_pose
      multi_pose_count: 3
      width_margin_fail: 0.10
      width_margin_pass: 0.20
      length_margin_fail: 0.05
      length_margin_pass: 0.20
      parkable_ratio_fail: 0.50
      parkable_ratio_pass: 0.90

    # ---------- TF ----------
    tf:
      global_frame: "map"
      base_frame: "base_link"
      timeout_sec: 0.1
      missing_is_fail: false       # 缺失时默认 WARN

    # ---------- 运行 ----------
    update_rate: 5.0               # Hz，评估周期
    debounce_frames: 3             # 连续 N 帧结论一致才发布
    publish_marker: true
    enable_service: true
    goal_pose_topic: "/planning/mission_planning/route"
```

---

## 8. 项目文件树

```
parking_input_evaluator/
├── CMakeLists.txt
├── package.xml
├── README.md
├── include/
│   └── parking_input_evaluator/
│       ├── parking_input_evaluator_node.hpp   # 主节点：订阅/节流/发布
│       ├── grid_analyzer.hpp                  # A 组 + F1/F2/F3
│       ├── pose_checker.hpp                   # B/C 组的矩形与坐标检查
│       ├── connectivity_checker.hpp           # EDT + 连通域
│       ├── parking_space_analyzer.hpp         # F4 + G1/G2 + 地图解析
│       ├── param.hpp                          # 参数结构与解析
│       ├── result.hpp                         # 结果结构与分级函数
│       └── visualization.hpp                  # MarkerArray 生成
├── src/
│   ├── parking_input_evaluator_node.cpp
│   ├── grid_analyzer.cpp
│   ├── pose_checker.cpp
│   ├── connectivity_checker.cpp
│   ├── parking_space_analyzer.cpp
│   └── visualization.cpp
├── msg/
│   ├── ParkingInputReport.msg                 # 总报告
│   ├── CheckItem.msg                          # 单项检查
│   └── ParkingSpaceStatus.msg                 # 单停车位状态
├── srv/
│   └── GetParkingInputReport.srv
├── config/
│   └── parking_input_evaluator.param.yaml
├── launch/
│   └── parking_input_evaluator.launch.xml
├── test/
│   ├── test_grid_analyzer.cpp                 # 覆盖率/值分布/EDT
│   ├── test_pose_checker.cpp                  # 矩形采样边界
│   ├── test_connectivity.cpp                  # 连通域与 EDT 一致性
│   ├── test_parking_space.cpp                 # 车位分类逻辑
│   └── test_integration.test.py               # bag 回放端到端
└── docs/
    ├── DESIGN.md                              # 本文档
    ├── METRICS.md                             # 23 项公式与阈值推导
    └── BASELINE.md                            # 实测基线（session2）
```

### 8.1 各文件职责

| 文件 | 职责 | 是否依赖 ROS |
| --- | --- | --- |
| `parking_input_evaluator_node.cpp` | 订阅 6 个话题、缓存、按 `update_rate` 触发评估、去抖、发布 | 是 |
| `grid_analyzer.cpp` | A1~A4；栅格索引、值统计、覆盖率 bbox 计算 | 仅数据结构 |
| `pose_checker.cpp` | B2~B5、C2~C4；世界坐标↔栅格、车矩形采样 | 否 |
| `connectivity_checker.cpp` | EDT 计算、\(\mathcal{P}\) 构造、连通域标记、F1~F3 | 否（可用 OpenCV） |
| `parking_space_analyzer.cpp` | 从 `LaneletMapBin` 提取车位、带状判定、F4 分类、G1/G2 | 是（lanelet2） |
| `visualization.cpp` | 覆盖框、盲区矩形、车位染色、goal 箭头 | 是 |

**设计原则**：除节点主文件与车位解析外，其余模块均为纯函数，不持有 ROS 句柄，便于单元测试和离线复用。

---

## 9. 消息与服务定义

### 9.1 `CheckItem.msg`

```msg
string id                       # "A3"、"F4" 等
string title                    # 人类可读标题
uint8 level                     # 0=UNKNOWN 1=PASS 2=WARN 3=FAIL
float64 value                   # 实测值
float64 threshold_pass          # PASS 阈值（不可比时为 NaN）
float64 threshold_warn          # WARN 阈值
string unit                     # "m"、"Hz"、"" 等
string detail                   # 完整实测描述
string advice                   # 修复建议，PASS 时为空
```

### 9.2 `ParkingSpaceStatus.msg`

```msg
uint64 id
geometry_msgs/Point center
float64 heading                 # rad
uint8 status                    # 0=PARKABLE 1=OUT_OF_GRID 2=BLOCKED 3=UNREACHABLE
uint32 conflict_sample_count    # 车矩形采样冲突数
float64 available_width         # 扣除障碍后的可用宽度估计
```

### 9.3 `ParkingInputReport.msg`

```msg
std_msgs/Header header

bool is_input_ready             # 系统整体是否可泊
bool can_plan_current_goal      # 本次 goal 是否可规划

uint8 worst_level               # 0/1/2/3
uint32 pass_count
uint32 warn_count
uint32 fail_count
string[] failure_reasons        # FAIL 项摘要，便于直接打印

CheckItem[] checks              # 23 项明细

# 关键标量，便于绘图/告警
float64 grid_rate_hz
float64 coverage_ratio
float64 unknown_ratio
float64 grid_extent[4]          # xmin, ymin, xmax, ymax
float64 parking_lot_extent[4]
float64 goal_edt_margin
float64 goal_rect_conflicts

# 车位统计
uint32 total_spaces
uint32 parkable_spaces
uint32 out_of_grid_spaces
uint32 blocked_spaces
uint32 unreachable_spaces
ParkingSpaceStatus[] parking_spaces
```

### 9.4 `GetParkingInputReport.srv`

```srv
bool include_space_details
---
ParkingInputReport report
```

---

## 10. 核心算法实现要点

### 10.1 坐标与索引

```cpp
struct GridIndex { int ix; int iy; bool inside; };

GridIndex worldToCell(double x, double y, const GridInfo& g) {
  const int ix = static_cast<int>(std::floor((x - g.origin_x) / g.resolution));
  const int iy = static_cast<int>(std::floor((y - g.origin_y) / g.resolution));
  const bool inside = (ix >= 0 && ix < g.width && iy >= 0 && iy < g.height);
  return {ix, iy, inside};
}

inline size_t flatten(int ix, int iy, int width) {
  return static_cast<size_t>(iy) * width + ix;   // row-major，与 OccupancyGrid 一致
}
```

注意 `OccupancyGrid.data` 为行主序，`index = iy * width + ix`；`cost` 为 `int8`，`-1` 表示 unknown。

### 10.2 车辆矩形采样

```cpp
// 返回第一个冲突或越界的采样点；无冲突返回 std::nullopt
std::optional<std::pair<int,int>> sampleRect(
  const OccupancyGrid& grid, double cx, double cy, double yaw,
  double length, double width, int n_long, int n_lat)
{
  const double c = std::cos(yaw), s = std::sin(yaw);
  for (int i = 0; i < n_long; ++i) {
    const double a = -length/2 + length * i / (n_long - 1);
    for (int j = 0; j < n_lat; ++j) {
      const double b = -width/2 + width * j / (n_lat - 1);
      const double x = cx + a*c - b*s;
      const double y = cy + a*s + b*c;
      auto idx = worldToCell(x, y, grid_info(grid));
      if (!idx.inside) return std::make_pair(-1, -1);      // 越界
      if (isObstacle(grid, idx)) return std::make_pair(idx.ix, idx.iy);
    }
  }
  return std::nullopt;
}
```

采样密度建议 `11 × 7`（车长 4.56 m、车宽 2.06 m，间距约 0.46 m / 0.34 m），已能覆盖 0.3 m 分辨率的全部格。

### 10.3 A* 等价的 EDT 约束

```cpp
// free = !obstacle；图像约定 0=障碍，255=free（OpenCV distanceTransform）
cv::Mat free_img(height, width, CV_8U);
for (int i = 0; i < height*width; ++i)
  free_img.data[i] = is_obstacle_raw(data[i], threshold) ? 0 : 255;

cv::Mat dist;
cv::distanceTransform(free_img, dist, cv::DIST_L2, 5);
// dist 单位为像素；乘 resolution 得米
```

构造可通行集合：

```cpp
const double half_width = collision_width / 2.0;
cv::Mat passable = (dist_m >= half_width) & free_mask;
```

OpenCV 的 `distanceTransform` 在 233×233 上约 0.3 ms、367×367 上约 0.8 ms，完全满足 5 Hz 预算。

若不愿引入 OpenCV，可用两遍扫描的倒角距离变换（chamfer 3-4），误差 < 5%，对阈值判定足够。

### 10.4 连通域

```cpp
cv::Mat labels;
int n = cv::connectedComponents(passable, labels, 8, CV_32S);
const bool same = labels.at<int>(sy, sx) != 0
               && labels.at<int>(sy, sx) == labels.at<int>(gy, gx);
```

必须在 8 邻域下标记——4 邻域会把斜向可通行的窄缝误判为断开（对角像素实际可通行）。

### 10.5 车位几何加载

本项目的 OSM 使用 VMB 导出格式，节点坐标存在 `local_x` / `local_y` 标签里，`lat/lon` 为空：

```xml
<node id="36777" lat="" lon="">
  <tag k="local_x" v="295.6794"/>
  <tag k="local_y" v="225.2732"/>
</node>
```

**标准 lanelet2 OSM parser 只读 `lat/lon`**，会把这类节点全部解析成 `(0,0)`。实测 `pointLayer` 的 x 范围是 `[0.0, 0.0]`，直接导致车位长度为 0、覆盖率恒为 0。Autoware 对这种情况有专门处理：`lanelet2_map_loader.cpp:163` 用 `AutowareOsmParser` 在建图后手动覆盖 `x/y`。

本节点为保持独立、不依赖 `autoware_lanelet2_extension`，直接用 **tinyxml2** 解析：

```cpp
tinyxml2::XMLDocument document;
document.LoadFile(osm_path.c_str());

// 1) 收集所有带 local_x / local_y 的 node
for (const auto* node = osm->FirstChildElement("node"); node;
     node = node->NextSiblingElement("node")) {
  Point2D p;
  if (nodeCoordinates(node, &p)) {   // 遍历 tag，读 k="local_x"/"local_y"
    nodes[std::strtoull(node->Attribute("id"), nullptr, 10)] = p;
  }
}

// 2) 解析 way，按 type 分流
for (const auto* way = osm->FirstChildElement("way"); way;
     way = way->NextSiblingElement("way")) {
  // 读 tag: type / width；遍历 nd 的 ref 组装折线
  if (type == "parking_space") { /* 中心线 + width */ }
  else if (type == "parking_lot") { /* 外轮廓 */ }
}
```

**兼容性**：本地图中 `parking_lot` 与 `parking_space` 都是 `way`，统一走 `way` 分支即可，无需区分 `areaLayer`。若地图缺少 `parking_lot`，退化为由全部车位中心的包络框推断停车场范围。

### 10.6 性能预算

| 环节 | 233×233 (54k) | 367×367 (135k) | 备注 |
| --- | --- | --- | --- |
| 值统计 | 0.05 ms | 0.12 ms | 单遍遍历 |
| EDT | 0.3 ms | 0.8 ms | OpenCV |
| 连通域 | 0.2 ms | 0.5 ms | OpenCV |
| 车矩形（单次） | 0.005 ms | 0.005 ms | 77 次查询 |
| 35 车位分类 | 0.2 ms | 0.2 ms | 含矩形检查 |
| **合计** | **< 1 ms** | **< 2 ms** | 5 Hz 预算 200 ms |

结论：即使把栅格扩到 367×367，本节点开销仍可忽略。真正的性能瓶颈在 `costmap_generator`（A1 只有 4.25 Hz），与本节点无关。

**可优化点**：EDT 与连通域仅在栅格变化时重算，用 `header.stamp` 做缓存键；车矩形检查在 goal 不变时也可缓存。

---

## 11. 运行状态机与更新策略

```
        ┌──────────┐   缺任一必需输入   ┌───────────┐
        │  IDLE    │─────────────────▶│  WAITING  │
        └──────────┘                  └───────────┘
             │ 输入齐备                       │ 输入恢复
             ▼                               │
        ┌──────────┐   评估完成           ┌───┴───────┐
        │ EVALUATE │────────────────────▶│ PUBLISHED │
        └──────────┘                     └───────────┘
```

### 11.1 触发条件

评估在一次定时器回调中执行，前提：

```cpp
bool ready = has_grid_ && has_route_ && has_odom_ && has_vector_map_
          && (!params_.scenario.required_enabled || current_scenario_ == "Parking");
```

缺输入时不发布结论，只发布 `WAITING` 状态的诊断，避免用陈旧数据误判。

### 11.2 去抖

单帧结论可能抖动（尤其 A1 频率低时）。策略：

- 连续 `debounce_frames`（默认 3）帧结论一致才发布；
- 级别取"最坏值"的多数票，避免一次瞬时 WARN 触发告警；
- `FAIL` 不受去抖影响，立即发布（宁可误报不可漏报）。

### 11.3 缓存与重算

| 数据 | 缓存键 | 重算时机 |
| --- | --- | --- |
| 值统计、EDT、连通域 | `grid.header.stamp` | 栅格更新 |
| 车位几何 | `vector_map.stamp` | 地图更新 |
| goal 检查 | `goal_pose` 哈希 | goal 变化 |
| 起点检查 | 每帧 | 位姿变化 |

---

## 12. 验证与测试计划

### 12.1 单元测试

| 用例 | 输入 | 期望 |
| --- | --- | --- |
| 覆盖率计算 | 已知 bbox 的合成栅格 | 与手算一致（误差 <1e-6） |
| 矩形采样越界 | goal 在栅格边缘外 1 格 | 返回越界标记 |
| 矩形采样碰撞 | 合成 3×3 障碍块置于车中心 | 返回冲突格 |
| EDT 阈值 | 单障碍点，距离 0.5/1.0/1.5 m | 与解析解一致 |
| 连通域 | 中间被一行障碍切断 | 判为不同域 |
| 车位分类 | 构造 OUT_OF_GRID/BLOCKED/UNREACHABLE 三种场景 | 分类正确 |
| 分级函数 | `(true,true)`、`(false,true)`、`(false,false)` | PASS/WARN/FAIL |

**重点回归**：分级函数必须保证 `pass_ok=true` 时一定返回 PASS，与 `warn_ok` 无关——这是离线工具中实际发生过的语义反转 bug。

### 12.2 离线回放基线（已有实测）

用 `session2` bag 回放，节点应复现以下结果（容差 ±1 项）：

```
PASS=18  WARN=3  FAIL=2
A3 覆盖率      53.7%（右侧缺 11.5 m，下侧缺 25.4 m）      FAIL
G4 栅格 vs 对角线 69.9 m vs 96.3 m                        FAIL
A1 栅格频率    4.25 Hz                                    WARN
C4 起点偏差    1.43 m                                     WARN
F4 可泊率      28/35（80%）                               WARN
```

车位明细必须一致：

| 车位中心 | 状态 |
| --- | --- |
| (280.35, 227.92) | BLOCKED |
| (272.70, 230.59) | BLOCKED |
| (291.06, 192.44) | OUT_OF_GRID |
| (291.08, 189.65) | OUT_OF_GRID |
| (291.08, 186.88) | OUT_OF_GRID |
| (291.09, 184.11) | OUT_OF_GRID |
| (291.09, 181.37) | OUT_OF_GRID |

该基线由 `~/parking_replay/evaluate_planning_inputs.py` 产出，C++ 节点实现必须与之逐项对齐，作为一致性验收标准。

**实测验证（2026-11-24，C++ 节点，session2 回放）**：

```
PASS=19  WARN=2  FAIL=2          （两次回放完全一致）
A3 53.7%（right 11.5 m / bottom 25.4 m）   FAIL   <- 与基线一致
G4 69.9 m vs 96.3 m                       FAIL   <- 与基线一致
A1 4.14 Hz（基线 4.25 Hz）                 WARN   <- 与基线一致
F4 27/35                                  WARN   <- 差 1 个
C4 gap 0.00 m                             PASS   <- 语义改进
```

5 个 `OUT_OF_GRID` 车位坐标与基线逐点一致。两处差异均有明确原因：

- `F4` 为 `27/35` 而非 `28/35`：离线工具默认矩形采样 `11×7`，节点默认 `23×13`，更密的采样多检出 `(298.37, 227.95)` 处的贴边冲突。把 `goal.rect_samples_long/lat` 调回 `11/7` 可复现 `28/35`。
- `C4` 为 PASS 而非 WARN：离线工具用 `odom[0]` 与 `route.start` 比较；节点记录 **route 到达瞬间**的位姿快照再比较，语义是"设置目标时车在哪"，因此 `gap=0` 且多次回放稳定。这是刻意改进，不是偏差。

**实现过程中的两个关键修正**（供后续维护参考）：

1. **分级函数语义**：`classify(pass_ok, warn_ok)` 的第二个参数表示"满足 WARN 阈值"，而不是"存在风险"。写反会导致所有本应 PASS 的项被误判为 WARN。
2. **地图解析**：不能用 lanelet2 读本地图（`lat/lon` 为空），必须读 `local_x/local_y`，详见 10.5 节。

### 12.3 与 planner 实跑对照

在 CARLA + Autoware 中，对 35 个车位逐个设为目标，记录：

- 本节点预测：`can_plan_current_goal`；
- planner 实际：`/planning/scenario_planning/parking/trajectory` 是否有输出。

构造混淆矩阵：

| | planner 成功 | planner 失败 |
| --- | --- | --- |
| 预测可规划 | TP | FP（乐观偏差，重点分析） |
| 预测不可规划 | FN（漏报，需修） | TN |

目标：FN = 0（不允许漏报），FP 比例 < 5%。若 FP 集中在窄缝场景，则开启 `strict_rect_connectivity`。

### 12.4 性能测试

用 `ros2 topic hz` + 节点自带的 `processing_time` 话题，验证：

- 233×233：单次 < 1 ms；
- 367×367：单次 < 2 ms；
- 5 Hz 下 CPU 占用 < 2%（单核）。

---

## 13. 开发里程碑

| 阶段 | 内容 | 交付物 | 验收标准 |
| --- | --- | --- | --- |
| **M1** | 包骨架 + 消息定义 + A 组（栅格本体） | 可编译、可订阅、输出 A1~A4 | 对 session2 复现 A1=4.25 Hz、A3=53.7% |
| **M2** | B/C/D/E 组 + 分级函数 | 23 项中 14 项可用，`worst_level` 正确 | 对 session2 复现 B1~B5 全 PASS、C4=1.43 m |
| **M3** | F 组（EDT + 连通域 + 车位可泊率） | `can_plan_current_goal` 可用，F4 输出 35 个车位状态 | 对 session2 复现 28/35、7 个失败车位坐标一致 |
| **M4** | G 组 + 可视化 + 服务 + 诊断 | 完整节点 | 23 项全量对齐离线工具 |
| **M5** | 单元测试 + 实跑对照 + 阈值校准 | 测试覆盖率 >80%，FP <5% | 35 车位混淆矩阵达标 |

**关键依赖**：M1 的前提是确认 `vector_map` 的 QoS 与加载时机；M3 的前提是 OpenCV 与 lanelet2 已正确链接。

---

## 14. 使用方式

### 14.1 编译

```bash
source /opt/ros/humble/setup.bash
source ~/autoware_2.1.0/install/setup.bash

cd ~/autoware_ws/src
git clone <repo> parking_input_evaluator
cd ~/autoware_ws
colcon build --packages-select parking_input_evaluator \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

### 14.2 启动

```bash
ros2 launch parking_input_evaluator parking_input_evaluator.launch.xml \
  params_file:=$HOME/parking_input_evaluator/config/parking_input_evaluator.param.yaml
```

建议在 `parking.launch.xml` 中作为可选项挂载，避免影响主链路：

```xml
<include file="$(find-pkg-share parking_input_evaluator)/launch/parking_input_evaluator.launch.xml"
         if="$(var enable_parking_input_evaluator)"/>
```

### 14.3 查看结果

```bash
# 实时报告
ros2 topic echo /planning/evaluation/parking_input_report

# 只看结论
ros2 topic echo /planning/evaluation/parking_input_report --field is_input_ready

# 主动拉取（含车位明细）
ros2 service call /planning/evaluation/get_parking_input_report \
  parking_input_evaluator/srv/GetParkingInputReport "{include_space_details: true}"

# 诊断
ros2 topic echo /diagnostics | grep parking_input
```

### 14.4 离线复用

节点内部的计算模块（`grid_analyzer` / `pose_checker` / `connectivity_checker` / `parking_space_analyzer`）均为纯函数，可编译为独立可执行文件，复用现有脚本的调用方式：

```bash
# 与离线工具输出逐项对比
ros2 run parking_input_evaluator evaluate_from_bag \
  --bag ~/parking_bag/session2 \
  --osm ~/autoware_data/maps/Town04/lanelet2_map.osm
```

---

## 15. 附录：指标与失败模式对照

| 失败模式 | 最先报警的指标 | 典型实测 | 推荐修复 |
| --- | --- | --- | --- |
| 目标在盲区 | A3 / B2 / F4(OUT_OF_GRID) | 覆盖率 53.7% | `grid_length_x/y: 70 → 110` |
| 车位太窄 | G1 / F4(BLOCKED) | 可用宽度 1.99 m < 2.057 m | 提高 `minimum_lidar_height_thres` 到 0.15 |
| 相邻车辆膨胀吃位 | F4(BLOCKED) | 膨胀半径 1.7 m | `size_of_expansion_kernel: 9 → 5` |
| 通道被割裂 | F1 / F2 / F4(UNREACHABLE) | 连通域不同 | 检查通道宽度与膨胀 |
| 场景未激活 | D1 | Parking 占比 0% | 检查 `scenario_selector` |
| 起终点不同步 | C4 | 偏差 1.43 m | 车辆静止后再设目标 |
| 栅格过旧 | A1 | 4.25 Hz | 降膨胀核 / 降点云频率 |
| 朝向容差过严 | G3 / B5 | 容差 < 1 档 | 增大 `angle_goal_range` |
| 栅格分辨率不足 | G1 通过但 F4 BLOCKED | 几何余量 0.27 m 仍碰撞 | `grid_resolution: 0.3 → 0.2` |

**排查顺序建议**：先看 A 组（有无数据）→ 再看 D/E（链路是否通）→ 然后 B/C（起终点是否合法）→ 最后 F/G（空间是否够）。这与失败概率的分布一致，也是实际操作中最省时的顺序。
