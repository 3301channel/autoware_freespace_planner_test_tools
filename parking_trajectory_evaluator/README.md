# parking_trajectory_evaluator

泊车**轨迹**的只读评估节点，订阅 `autoware_freespace_planner` 的输出，
判断这条轨迹能否安全、准确、舒适地被控制器执行。

```
┌──────────────┐   ┌──────────────────┐   ┌────────────────────┐
│ 上游输入     │──▶│ input_evaluator  │──▶│ freespace_planner  │
│ grid/route   │   │ is_input_ready   │   │      A*            │
└──────────────┘   └──────────────────┘   └─────────┬──────────┘
                                                     │ trajectory
                                                     ▼
                                         ┌──────────────────────┐
                                         │ trajectory_evaluator │
                                         │  is_control_ready    │
                                         └──────────────────────┘
```

---

## 1. 构建

与 `parking_input_evaluator` 相同，源码在中文路径下，通过符号链接到 ASCII 工作区构建。

```bash
mkdir -p ~/parking_tools_ws/src
ln -sfn "$HOME/桌面/tools/parking_trajectory_evaluator" \
        ~/parking_tools_ws/src/parking_trajectory_evaluator

source /opt/ros/humble/setup.bash
source ~/autoware_2.1.0/install/setup.bash

cd ~/parking_tools_ws
colcon build --packages-select parking_trajectory_evaluator \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DPYTHON_EXECUTABLE=/usr/bin/python3

source install/setup.bash
```

---

## 2. 运行

```bash
ros2 launch parking_trajectory_evaluator parking_trajectory_evaluator.launch.xml \
  use_sim_time:=true
```

### 话题可配置

与输入评估节点一致，栅格来源可切换（`costmap_generator` 或直接生成 occ 的节点）：

```bash
ros2 launch parking_trajectory_evaluator parking_trajectory_evaluator.launch.xml \
  occupancy_grid_topic:=/your/occupancy_grid_topic \
  use_sim_time:=true
```

| 参数 | 默认值 |
| --- | --- |
| `trajectory_topic` | `/planning/scenario_planning/parking/trajectory` |
| `odometry_topic` | `/localization/kinematic_state` |
| `scenario_topic` | `/planning/scenario_planning/scenario` |
| `occupancy_grid_topic` | `/planning/scenario_planning/parking/costmap_generator/occupancy_grid` |

---

## 3. 输出

```bash
# 完整报告
ros2 topic echo /planning/evaluation/parking_trajectory_report

# 只看结论与分数
ros2 topic echo /planning/evaluation/parking_trajectory_report --field is_control_ready
ros2 topic echo /planning/evaluation/parking_trajectory_report --field total_score

# 主动拉取
ros2 service call /planning/evaluation/get_parking_report \
  parking_trajectory_evaluator/srv/GetParkingReport "{include_metrics: true}"

# 诊断
ros2 topic echo /diagnostics | grep -A6 parking_trajectory_evaluator
```

RViz 添加 `MarkerArray`，话题 `/planning/evaluation/parking_trajectory_marker`：
绿=可控、红=不可控、蓝色段=倒车、红球=碰撞点、箭头=终点。

---

## 4. 指标与判定

### 硬约束（任一失败即 `is_control_ready=false`）

| 指标 | 判据 |
| --- | --- |
| 点数 | `N ≥ min_points` |
| 有限性 | 无 NaN/Inf |
| 点间距 | `0.01 ≤ Δs ≤ 2.0 m` |
| 曲率 | `|κ| ≤ 0.398 1/m`（`R_min = L/tan(δ_max)`） |
| 转向角 | `|δ| ≤ 0.70 rad` |
| 转向角速率 | `|δ̇| ≤ 0.3 rad/s` |
| 速度 | `|v| ≤ 1.4 m/s` |
| 终点速度 | `|v_N| ≤ 0.05 m/s` |
| 朝向跳变 | `|Δθ| ≤ 0.5 rad` |
| 中心碰撞 | `= 0` |
| 外廓碰撞 | `= 0` |
| 末端横向 | `≤ 0.3 m` |
| 末端纵向 | `≤ 0.5 m` |
| 末端航向 | `≤ 0.1745 rad` |

### 软约束评分（100 分）

| 项 | 权重 | 规则 |
| --- | --- | --- |
| 末端横向误差 | 20 | 0 → 满分，线性衰减到容差 → 0 |
| 末端纵向误差 | 15 | 同上 |
| 末端航向误差 | 15 | 同上 |
| 换向次数 | 15 | ≤2 满分，每多 1 次扣 5 |
| 最大横向加速度 | 10 | ≤2.0 满分，衰减到 9.8 → 0 |
| 最大横向 jerk | 10 | ≤1.5 满分，衰减到 7.0 → 0 |
| 最小障碍间隙 | 10 | ≥0.5 m 满分，0 → 0 |
| 曲率方差 | 5 | 越小越高 |

`is_control_ready = 无硬约束失败 AND total_score ≥ 70`

> 未启用 occupancy 时跳过碰撞与间隙项；未配置 `goal` 时末端三项给满分（不惩罚缺失信息），
> 报告中 `goal_configured=false` 表明精度未评估。

---

## 5. 实测验证（2026-11-24）

用合成轨迹验证（无 bag 含 `parking/trajectory`）：

| 场景 | is_control_ready | score | 失败原因 |
| --- | --- | --- | --- |
| 正常轨迹（0.5 m/s，终点 0，朝向匹配） | **true** | **97.1** | — |
| 超速轨迹（3.0 m/s） | **false** | 97.1 | `velocity exceeds the parking limit` |

超速轨迹的 `max_abs_velocity=3.0`、`terminal_velocity=0.0`，说明节点正确区分了
"巡航超速"与"终点静止"两个独立约束。

单元测试 **23 项全部通过**，覆盖：
曲率与圆弧半径一致性、自行车模型转向角、NaN 检测、换向计数、间距异常、
碰撞（中心/外廓/出界）、评分单调性、各硬约束边界。

---

## 6. 与输入评估节点配合

```bash
# 终端 1
ros2 launch parking_input_evaluator parking_input_evaluator.launch.xml use_sim_time:=true

# 终端 2
ros2 launch parking_trajectory_evaluator parking_trajectory_evaluator.launch.xml use_sim_time:=true
```

排查顺序：

1. `is_input_ready=false` → 先修输入（栅格盲区、车位过窄），此时 planner 大概率失败；
2. `is_input_ready=true` 但 `is_control_ready=false` → 输入没问题，是 planner 输出质量问题
   （曲率超限、末端偏差、碰撞）。

这样能把"规划失败"的责任清晰地分到输入侧或输出侧。

---

## 7. 目录

```
parking_trajectory_evaluator/
├── include/parking_trajectory_evaluator/
│   ├── param.hpp                 # 参数
│   ├── result.hpp                # 轨迹/指标/评分结构
│   ├── trajectory_metrics.hpp    # 指标计算
│   ├── collision_checker.hpp     # 栅格碰撞
│   ├── scoring.hpp               # 判定与评分
│   └── visualization.hpp
├── src/                          # 实现 + 节点
├── msg/                          # MetricValue / ParkingTrajectoryReport
├── srv/                          # GetParkingReport
├── config launch test
```
