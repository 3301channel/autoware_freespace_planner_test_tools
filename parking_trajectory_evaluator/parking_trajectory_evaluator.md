# parking_trajectory_evaluator 开发文档

> 目标版本：ROS 2 Humble + Autoware 2.1.0
> 用途：对autoware泊车规划器freespace_planner输出进行评估（监听话题`/planning/scenario_planning/parking/trajectory`）

## 1. 项目目标

### 1.1 背景

Autoware 2.1.0 的 `autoware_freespace_planner` 在 `PARKING` 场景下生成泊车轨迹，输出到 `/planning/scenario_planning/parking/trajectory`。但官方自带的评估/验证模块存在以下不足：

- `planning_evaluator` 默认参考轨迹指向车道规划输出，无泊车专用指标；
- `planning_validator` / `TrajectoryChecker` 只有通用的汽车约束检查，无泊车专属逻辑；
- 没有任何模块对"这条泊车轨迹最终能否被控制器安全、准确地执行"给出明确结论。

因此需要开发一个**独立、只读、可插拔**的评估节点，订阅 freespace_planner 的轨迹，输出量化的评估结果和"是否满足控制阶段泊车"的判定。

### 1.2 核心目标

1. **只读评估**：节点只订阅话题，不发布控制指令、不修改任何规划轨迹，对现有系统零侵入。
2. **科学严谨**：基于车辆运动学/动力学模型，计算可解释的量化指标，而非简单阈值判断。
3. **控制可判**：给出一个明确的布尔结论 `is_control_ready`，说明当前轨迹是否满足进入控制阶段的条件，并附带失败原因。
4. **可配置**：车辆参数、阈值、目标车位位置均通过 ROS2 参数配置，不硬编码。
5. **可观测**：通过 topic / service / 日志三种方式输出评估详情，便于调试和离线分析。

### 1.3 非目标（Non-goals）

- 不生成、不修改轨迹；
- 不做障碍物检测或占用栅格计算（依赖上游 costmap）；
- 不做端到端自动驾驶决策；
- 不与 scenario_selector / controller 抢控制权。

---

## 2. 输入与输出接口

### 2.1 订阅话题（Input）

| 话题 | 类型 | 用途 | 是否必须 |
| --- | --- | --- | --- |
| `/planning/scenario_planning/parking/trajectory` | `autoware_planning_msgs/msg/Trajectory` | 待评估的泊车轨迹（freespace_planner 输出） | 必须 |
| `/localization/kinematic_state` | `nav_msgs/msg/Odometry` | 当前车辆位姿/速度，用于计算起点误差 | 必须 |
| `/planning/scenario_planning/scenario` | `tier4_planning_msgs/msg/Scenario` | 确认当前确实是 PARKING 场景（可选过滤） | 可选 |
| `/planning/scenario_planning/parking/costmap_generator/occupancy_grid` | `nav_msgs/msg/OccupancyGrid` | 轨迹安全性：是否侵入障碍物区域 | 可选 |
| `/vehicle/status/steering_status` | `autoware_vehicle_msgs/msg/SteeringStatus` | 当前转向角，用于可执行性评估 | 可选 |

### 2.2 发布话题（Output）

| 话题 | 类型 | 内容 |
| --- | --- | --- |
| `/planning/evaluation/parking_trajectory_report` | 自定义消息 `ParkingTrajectoryReport` | 完整评估结果：各指标值、布尔判定、失败原因列表 |
| `/planning/evaluation/parking_trajectory_marker` | `visualization_msgs/msg/MarkerArray` | RViz 可视化：轨迹染色（绿=安全/红=不合格）、关键点标注 |
| `/diagnostics` | `diagnostic_msgs/msg/DiagnosticArray` | 标准化诊断状态 |

### 2.3 服务（Service）

| 服务 | 类型 | 用途 |
| --- | --- | --- |
| `/planning/evaluation/get_parking_report` | 自定义 `GetParkingReport.srv` | 主动拉取最近一次评估报告 |

### 2.4 参数（ROS2 Parameters）

见第 6 节"参数设计"。

---

## 3. 评估指标体系

设轨迹由 \(N\) 个点组成，第 \(i\) 个点位姿 \(p_i=(x_i,y_i,\theta_i)\)，速度 \(v_i\)，相邻点弧长 \(\Delta s_i=\|p_{i+1}-p_i\|\)。

### 3.1 可行性指标

#### (1) 轨迹连续性 / 点间距
相邻点间距应在合理范围：
\[
\Delta s_{\min} \le \Delta s_i \le \Delta s_{\max}
\]
过大会导致控制离散化误差，过小浪费计算。默认值 \(\Delta s_{\min}=0.01\,\text{m}\)、\(\Delta s_{\max}=2.0\,\text{m}\)。

#### (2) 曲率约束（最小转弯半径）
轨迹曲率：
\[
\kappa_i = \frac{\Delta\theta_i}{\Delta s_i} = \frac{\text{wrap}(\theta_{i+1}-\theta_i)}{\Delta s_i}
\]
对应转弯半径 \(R_i = 1/|\kappa_i|\)。车辆最小转弯半径：
\[
R_{\min} = \frac{L}{\tan\delta_{\max}}
\]
其中 \(L=2.831\,\text{m}\)（轴距），\(\delta_{\max}=0.70\,\text{rad}\)（最大转向角）。要求：
\[
|\kappa_i| \le \kappa_{\max} = \frac{1}{R_{\min}} \approx 0.398\,\text{m}^{-1}
\]
**注意**：freespace_planner 自身只通过 `max_turning_ratio` 间接约束曲率，本节点显式检查。

#### (3) 转向角 / 转向角速率
由自行车模型：
\[
\delta_i = \arctan(L\kappa_i), \qquad \dot\delta_i = \frac{\delta_{i+1}-\delta_i}{\Delta t_i}
\]
要求 \(|\delta_i| \le \delta_{\max}\)，\(|\dot\delta_i| \le \dot\delta_{\max}\)（默认 \(0.3\,\text{rad/s}\)，参考 trajectory_validator 配置）。

#### (4) 速度约束
\[
|v_i| \le v_{\max,\text{parking}} \quad (\text{默认 } 1.4\,\text{m/s} \approx 5\,\text{km/h})
\]
倒车段 \(v_i<0\) 应与 `is_back` 标记一致（可从相邻点朝向变化推断）。

#### (5) 轨迹终点静止
泊车终点速度必须为零：
\[
|v_N| \le v_{\varepsilon} \quad (\text{默认 } 0.05\,\text{m/s})
\]

#### (6) 轨迹有效性
无 NaN / Inf；点数 \(N\ge N_{\min}\)（默认 5）。

### 3.2 安全性指标

#### (7) 障碍物间隙（需 occupancy_grid）
对轨迹上每个点，查询其对应栅格及周围膨胀区的代价值。要求轨迹点不在占用栅格的障碍区域内：
\[
\text{cost}(p_i) < \text{cost}_{\text{threshold}}
\]
统计侵入次数 \(N_{\text{collision}}\)，要求 \(N_{\text{collision}}=0\)。

#### (8) 车辆外廓碰撞（需 occupancy_grid + 车辆尺寸）
考虑车辆矩形外廓（长 \(4.5\,\text{m}\)、宽 \(1.8\,\text{m}\)、轴距 \(2.831\,\text{m}\)），在每个轨迹点按朝向旋转后采样角点，检查是否侵入障碍。比仅检查中心点更严格。

### 3.3 控制适配性指标

#### (9) 末端位姿精度
若配置了目标车位位姿 \(p_{\text{goal}}=(x_g,y_g,\theta_g)\)：
\[
e_{\text{lat}} = |(p_N-p_g)\cdot \hat{n}|,\quad e_{\text{lon}} = |(p_N-p_g)\cdot \hat{t}|,\quad e_{\theta}=|\text{wrap}(\theta_N-\theta_g)|
\]
要求 \(e_{\text{lat}}\le 0.3\,\text{m}\)、\(e_{\text{lon}}\le 0.5\,\text{m}\)、\(e_{\theta}\le 10^\circ\)。

#### (10) 换向次数
统计速度符号变化次数：
\[
N_{\text{switch}} = |\{i : \text{sign}(v_i)\ne \text{sign}(v_{i-1}),\, |v_i|>v_\varepsilon\}|
\]
次数越少轨迹越优雅。默认阈值 \(N_{\text{switch}}\le 4\)。

#### (11) 横向加速度 / 横向 jerk（舒适性）
\[
a_{\text{lat},i} = v_i^2 \kappa_i, \qquad j_{\text{lat},i} \approx \frac{a_{\text{lat},i+1}-a_{\text{lat},i}}{\Delta t_i}
\]
阈值参考 planning_validator：\(|a_{\text{lat}}|\le 9.8\,\text{m/s}^2\)，\(|j_{\text{lat}}|\le 7.0\,\text{m/s}^3\)。泊车场景建议更严格（如 \(2.0\,\text{m/s}^2\)、\(1.5\,\text{m/s}^3\)）。

#### (12) 纵向加速度 / jerk（需 smoother 后的轨迹）
由于 freespace_planner 输出的加速度/jerk 为 0，本节点**自行基于速度差分估算**：
\[
a_{\text{lon},i} = \frac{v_{i+1}-v_i}{\Delta t_i}, \qquad j_{\text{lon},i} = \frac{a_{i+1}-a_i}{\Delta t_i}
\]
其中 \(\Delta t_i = \Delta s_i / \max(|v_i|, v_{\min})\)。阈值参考 PID 控制器：\(|a_{\text{lon}}|\le 3.0\,\text{m/s}^2\)，\(|j_{\text{lon}}|\le 2.0\,\text{m/s}^3\)。

#### (13) 轨迹跳变
相邻点之间位姿突变检测：
\[
\|p_{i+1}-p_i\| \le \Delta s_{\max}, \qquad |\text{wrap}(\theta_{i+1}-\theta_i)| \le \Delta\theta_{\max}
\]
默认 \(\Delta\theta_{\max}=0.5\,\text{rad}\)。

---

## 4. 判定逻辑

### 4.1 总体结论

节点输出 `is_control_ready: bool`，判定规则：

```
is_control_ready = 所有硬约束(3.1节 + 3.2节)通过
                   AND 软约束评分 >= 阈值(默认 70/100)
```

### 4.2 硬约束

任一失败即将 `is_control_ready=false`，并将失败原因写入 `failure_reasons` 数组：

- 轨迹连续性失败
- 曲率超限
- 转向角/角速度超限
- 速度超限
- 终点未静止
- 数据含 NaN/Inf
- 轨迹点侵入障碍物
- 车辆外廓碰撞

### 4.3 软约束评分

| 指标 | 权重（total:100） | 得分规则 |
| --- | --- | --- |
| 末端横向误差 | 20 | 误差 0 得满分，线性衰减到阈值处得 0 |
| 末端纵向误差 | 15 | 同上 |
| 末端航向误差 | 15 | 同上 |
| 换向次数 | 15 | \(\le 2\) 满分，每多 1 次扣 5 分 |
| 最大横向加速度 | 10 | 低于舒适阈值满分，线性衰减 |
| 最大横向 jerk | 10 | 同上 |
| 最小障碍物间隙 | 10 | 大于安全距离满分，线性衰减 |
| 轨迹平滑度（曲率方差） | 5 | 方差越小分越高 |

总分 \(\ge\) `min_score`（默认 70）且无硬约束失败 → `is_control_ready=true`。

### 4.4 分场景判定

考虑到 freespace_planner 输出**单方向分段轨迹**，节点支持两种模式：

- **单段模式**（默认）：每次评估一段轨迹，只判断该段是否可控；
- **累计模式**：缓存连续多段轨迹，拼接后评估整体泊车过程（用于完整泊车评估）。

由参数 `evaluation_mode` 控制。

---

## 5. 限制与假设

### 5.1 上游轨迹的固有限制

1. **freespace_planner 输出的是"半成品"**：只有 `pose` 和常量 `longitudinal_velocity_mps`，加速度/前轮转角/`time_from_start` 全为 0。本节点通过有限差分自行估算加速度和 jerk，精度低于 velocity_smoother 输出。
2. **单方向输出**：一次只输出前进或后退段，无法一次评估完整泊车过程。
3. **无曲率显式约束**：planner 只通过 `max_turning_ratio` 间接限制，需要本节点显式检查。

### 5.2 评估节点的限制

1. **只读不控制**：评估节点不阻止不合格轨迹进入控制器，仅输出诊断。需要人工或上层系统调用 `/api/planning/...` 来处理。
2. **加速度估算精度**：基于速度差分的加速度估算受点间距不均匀影响，只作参考。
3. **障碍物评估依赖 costmap**：未提供 occupancy_grid 时跳过安全指标，判定结果标注"安全性未评估"。
4. **目标车位位姿需配置**：若未设置目标车位，跳过末端精度指标。
5. **不评估控制跟踪误差**：本节点评估的是"规划轨迹"本身，不是实际执行效果。

### 5.3 环境依赖

- ROS2 Humble
- Autoware 2.1.0（`autoware_planning_msgs`、`autoware_vehicle_msgs`、`tier4_planning_msgs` 等消息包）
- C++17、rclcpp、tf2_ros
- 可选：OpenCV（RViz 可视化）

---

## 6. 参数设计

```yaml
# parking_trajectory_evaluator.param.yaml
/**:
  ros__parameters:
    # 车辆参数（与 sample_vehicle 一致）
    vehicle:
      wheel_base: 2.831
      max_steer_angle: 0.70
      vehicle_length: 4.5
      vehicle_width: 1.8
      rear_overhang: 1.1

    # 硬约束阈值
    constraints:
      min_point_interval: 0.01
      max_point_interval: 2.0
      max_curvature: 0.398          # 1/R_min, R_min = L/tan(delta_max)
      max_steer_angle_rate: 0.3     # rad/s
      max_velocity: 1.4             # m/s
     终点速度阈值: 0.05
      min_points: 5
      max_relative_angle: 0.5       # rad
      occupancy_cost_threshold: 50

    # 软约束阈值
    comfort:
      max_lateral_acc: 2.0
      max_lateral_jerk: 1.5
      max_longitudinal_acc: 3.0
      max_longitudinal_jerk: 2.0
      max_switch_count: 4

    # 目标车位（可选，需按实际地图填写）
    goal:
      x: 273.215
      y: 236.173
      yaw: 3.14159                  # 180.2 deg
      tolerance_lat: 0.3
      tolerance_lon: 0.5
      tolerance_yaw: 0.1745         # 10 deg

    # 评分
    scoring:
      min_score: 70.0

    # 模式
    evaluation_mode: "single"       # "single" or "cumulative"
    enable_occupancy_check: true
    enable_visualization: true
    publish_rate: 10.0              # Hz
```

---

## 7. 项目文件树

```
parking_trajectory_evaluator/
├── CMakeLists.txt
├── package.xml
├── README.md
├── include/
│   └── parking_trajectory_evaluator/
│       ├── parking_trajectory_evaluator_node.hpp   # 主节点类定义
│       ├── trajectory_metrics.hpp                  # 指标计算函数
│       ├── evaluation_result.hpp                   # 结果结构体
│       ├── collision_checker.hpp                   # 障碍物碰撞检查
│       ├── scoring.hpp                             # 评分逻辑
│       └── visualization.hpp                       # RViz 可视化
├── src/
│   ├── parking_trajectory_evaluator_node.cpp       # 节点主逻辑：订阅、回调、发布
│   ├── trajectory_metrics.cpp                      # 曲率/加速度/jerk/误差计算
│   ├── collision_checker.cpp                       # occupancy grid 查询
│   ├── scoring.cpp                                 # 硬约束判断+软约束评分
│   └── visualization.cpp                           # MarkerArray 生成
├── msg/
│   ├── ParkingTrajectoryReport.msg                 # 评估报告消息
│   └── MetricValue.msg                             # 单个指标值（名称/值/阈值/是否通过）
├── srv/
│   └── GetParkingReport.srv                        # 拉取报告服务
├── config/
│   └── parking_trajectory_evaluator.param.yaml     # 参数配置
├── launch/
│   └── parking_trajectory_evaluator.launch.xml     # 启动文件
├── test/
│   ├── test_trajectory_metrics.cpp                 # 指标计算单元测试
│   ├── test_scoring.cpp                            # 评分逻辑单元测试
│   └── test_integration.test.py                    # 集成测试（ros2 launch + 发布模拟轨迹）
└── docs/
    ├── DESIGN.md                                   # 本文档
    └── METRICS.md                                  # 指标公式与阈值详解
```

### 7.1 各文件职责

| 文件 | 职责 |
| --- | --- |
| `parking_trajectory_evaluator_node.cpp` | 订阅 5 个话题、管理回调、触发评估、发布结果 |
| `trajectory_metrics.cpp` | 纯数学计算：曲率、加速度、jerk、误差、连续性；不涉及 ROS |
| `collision_checker.cpp` | 将轨迹点/车辆外廓映射到 occupancy grid 坐标系，查询代价 |
| `scoring.cpp` | 根据阈值判断硬约束、计算软约束分数、生成失败原因 |
| `visualization.cpp` | 生成彩色轨迹 Marker |
| `ParkingTrajectoryReport.msg` | 结构化输出所有指标、判定、失败原因 |

---

## 8. 消息定义草案

### 8.1 `MetricValue.msg`
```msg
string name           # 指标名，如 "max_curvature"
float64 value         # 实测值
float64 threshold     # 阈值
bool passed           # 是否通过
string unit           # 单位
```

### 8.2 `ParkingTrajectoryReport.msg`
```msg
std_msgs/Header header
bool is_control_ready                   # 总体结论
string[] failure_reasons                # 硬约束失败原因
float64 total_score                     # 软约束总分 (0-100)
MetricValue[] metrics                   # 所有指标明细
geometry_msgs/Pose terminal_pose        # 轨迹终点
float64 terminal_lat_error              # 末端横向误差
float64 terminal_lon_error              # 末端纵向误差
float64 terminal_yaw_error              # 末端航向误差
int32 switch_count                      # 换向次数
bool occupancy_evaluated                # 是否评估了障碍物
```

---

## 9. 验证与测试计划

### 9.1 单元测试
- 给定已知轨迹（圆弧、直线、S 形），验证曲率计算正确性；
- 验证硬约束边界（刚好等于阈值时通过/失败）；
- 验证评分函数的单调性和边界。

### 9.2 集成测试
- 用 `ros2 bag` 录制一段真实泊车轨迹，离线回放验证节点输出；
- 在 RViz 中目视检查 Marker 颜色与实际轨迹是否一致；
- 人为构造不合格轨迹（曲率超限、终点速度非 0），验证判定为 false。

### 9.3 实车/仿真验证
- 在 CARLA + Autoware 完整系统中运行；
- 对比评估节点结论与实际控制跟踪效果，校准阈值。

---

## 10. 开发节点

| 阶段 | 内容 | 交付物 |
| --- | --- | --- |
| M1 | 消息定义 + 基础节点框架 + 曲率/连续性指标 | 可编译的 ROS2 包，能订阅并输出基础报告 |
| M2 | 加速度/jerk/末端误差/换向次数 + 评分逻辑 | 完整指标体系，`is_control_ready` 可用 |
| M3 | occupancy grid 碰撞检查 + 可视化 | 安全性评估 + RViz Marker |
| M4 | 单元测试 + 集成测试 + 参数调优文档 | 测试通过，阈值可在真实数据上校准 |

---

## 11. 使用方式

```bash
source /opt/ros/humble/setup.bash
source ~/autoware_2.1.0/install/setup.bash

ros2 launch parking_trajectory_evaluator parking_trajectory_evaluator.launch.xml \
  params_file:=/path/to/parking_trajectory_evaluator.param.yaml

# 查看评估结果
ros2 topic echo /planning/evaluation/parking_trajectory_report

# 主动拉取
ros2 service call /planning/evaluation/get_parking_report \
  parking_trajectory_evaluator/srv/GetParkingReport "{}"
```
