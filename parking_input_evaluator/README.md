# parking_input_evaluator

泊车规划**上游输入**的只读评估节点。

它回答两个问题：

| 输出 | 含义 |
| --- | --- |
| `is_input_ready` | 系统是否处于稳定可泊状态（无 FAIL、无 WARN） |
| `can_plan_current_goal` | **本次 route 的 goal** 是否可规划 |

23 项检查覆盖栅格、目标、起点、场景、TF、连通性、车位几何，失败时给出可执行的修复建议。

---

## 1. 构建

> **注意**：rosidl 在包含中文的路径下会丢失路径片段（表现为 `/tools/build/...`），
> 因此源码保留在 `~/桌面/tools/`，通过符号链接到 ASCII 工作区构建。

```bash
# 一次性建立 ASCII 工作区
mkdir -p ~/parking_tools_ws/src
ln -sfn "$HOME/桌面/tools/parking_input_evaluator" \
        ~/parking_tools_ws/src/parking_input_evaluator

source /opt/ros/humble/setup.bash
source ~/autoware_2.1.0/install/setup.bash

cd ~/parking_tools_ws
colcon build --packages-select parking_input_evaluator \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DPYTHON_EXECUTABLE=/usr/bin/python3

source install/setup.bash
```

依赖：`rclcpp`、`tf2_ros`、OpenCV、Eigen3、tinyxml2、`autoware_planning_msgs`、
`autoware_map_msgs`、`autoware_internal_planning_msgs`。

---

## 2. 运行

```bash
ros2 launch parking_input_evaluator parking_input_evaluator.launch.xml \
  lanelet2_map_path:=$HOME/autoware_data/maps/Town04/lanelet2_map.osm \
  use_sim_time:=true
```

### 栅格换源

####################################################################################################

`freespace_planner` 的输入栅格可能来自 `costmap_generator`，也可能来自**直接生成
occupancy grid 的节点**。launch 暴露了 `occupancy_grid_topic`，无需改代码：

```bash
ros2 launch parking_input_evaluator parking_input_evaluator.launch.xml \
  occupancy_grid_topic:=/perception/occupancy_grid_map/map \
  lanelet2_map_path:=... use_sim_time:=true
```

| 参数 | 默认值 |
| --- | --- |
| `occupancy_grid_topic` | `/planning/scenario_planning/parking/costmap_generator/occupancy_grid` |
| `route_topic` | `/planning/mission_planning/route` |
| `odometry_topic` | `/localization/kinematic_state` |
| `scenario_topic` | `/planning/scenario_planning/scenario` |

---

## 3. 输出

```bash
# 完整报告
ros2 topic echo /planning/evaluation/parking_input_report

# 只取结论
ros2 topic echo /planning/evaluation/parking_input_report --field is_input_ready
ros2 topic echo /planning/evaluation/parking_input_report --field can_plan_current_goal

# 车位明细
ros2 service call /planning/evaluation/get_parking_input_report \
  parking_input_evaluator/srv/GetParkingInputReport "{include_space_details: true}"

# 诊断（FAIL -> ERROR，WARN -> WARN）
ros2 topic echo /diagnostics | grep -A5 parking_input_evaluator
```

RViz 中添加 `MarkerArray`，话题 `/planning/evaluation/parking_input_marker`：
蓝色框为栅格范围，黄线为停车场轮廓，车位按状态染色（绿可停 / 红碰撞 / 灰盲区 / 橙不可达），
箭头为 goal。

---

## 4. 实测基线（session2）

```
PASS=19  WARN=2  FAIL=2      （两次回放结果完全一致）
```

| 项 | 结果 | 说明 |
| --- | --- | --- |
| A3 覆盖率 | **53.7% FAIL** | 右缺 11.5 m、下缺 25.4 m |
| G4 栅格 vs 对角线 | **69.9 vs 96.3 m FAIL** | 结构性盲区 |
| A1 栅格频率 | **4.14 Hz WARN** | 期望 10 Hz |
| F4 可泊率 | **27/35 WARN** | 5 个在栅格外、3 个碰撞 |

5 个 `OUT_OF_GRID` 车位坐标：`(291.06,192.44) (291.08,189.65) (291.08,186.88)
(291.09,184.11) (291.09,181.37)`，与离线工具 `evaluate_planning_inputs.py` 完全一致。

### 与离线工具的已知差异

1. **F4 为 `27/35`，离线工具报 `28/35`**：离线工具默认车辆矩形采样 `11×7`，
   本节点默认 `23×13`，更密的采样多检出 `(298.37, 227.95)` 处的贴边冲突。
   把 `goal.rect_samples_long/lat` 调回 `11/7` 可复现 `28/35`。
2. **C4 为 PASS，离线工具为 WARN（1.4 m）**：离线工具拿 `odom[0]` 与
   `route.start` 比较；本节点记录 **route 到达瞬间**的位姿快照再比较，
   语义是"设目标时车在哪"，因此 `gap=0` 且两次回放稳定。这是刻意的改进。

---

## 5. 关键实现说明

### 5.1 为什么不用 lanelet2 读地图

本项目的 OSM 使用 VMB 风格：

```xml
<node id="36777" lat="" lon="">
  <tag k="local_x" v="295.6794"/>
  <tag k="local_y" v="225.2732"/>
</node>
```

`lat/lon` 为空，坐标存在 `local_x/local_y` 标签里。标准 lanelet2 OSM parser
只认 `lat/lon`，会把所有点解析成 `(0,0)`。Autoware 为此提供了
`AutowareOsmParser`（`lanelet2_map_loader.cpp:163` 手动覆盖 `x/y`）。

本节点为保持独立，直接用 **tinyxml2** 读取 `local_x/local_y`，不依赖 lanelet2。

### 5.2 与 A* 的语义对齐

| 环节 | 源码 | 实现 |
| --- | --- | --- |
| 障碍判定 | `abstract_algorithm.cpp:117-121` | `cost < 0 \|\| cost >= threshold` |
| 节点约束 | `astar_search.cpp:262` | EDT ≥ 半车宽（`analyzeConnectivity`） |
| 碰撞形状 | `freespace_planner_node.cpp:442-445` | `length += m`、`width += m`（只加一次） |

---

## 6. 测试

```bash
cd ~/parking_tools_ws
colcon test --packages-select parking_input_evaluator
colcon test-result --verbose
```

16 个单元测试覆盖：栅格索引、障碍规则、矩形碰撞、EDT、连通域、车位几何。

---

## 7. 目录

```
parking_input_evaluator/
├── include/parking_input_evaluator/   # param/result/grid/connectivity/map/pose/visualization
├── src/                               # 节点 + 算法实现
├── msg/                               # CheckItem / ParkingSpaceStatus / ParkingInputReport
├── srv/                               # GetParkingInputReport
├── config/                            # 参数
├── launch/                            # 启动文件
└── test/                              # 单元测试
```
