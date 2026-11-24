# nav_msgs/OccupancyGrid 消息解析

> 对应文件：`~/parking_replay/formats/nav_msgs_msg_OccupancyGrid.txt`
> 用途：理解 `freespace_planner` 泊车路径搜索所使用的二维栅格地图。

---

## 1. 概述

`nav_msgs/OccupancyGrid` 是 ROS 标准的**二维占据栅格地图**消息，在 Autoware 泊车场景中，它是 `freespace_planner` 进行 A* / Hybrid A* 路径搜索的"地图底图"。

消息结构分三部分：

```
header    ← 时间戳和坐标系
info      ← 栅格元信息（分辨率、尺寸、原点）
data      ← 栅格占用数据（一维数组）
```

---

## 2. 完整消息定义

```msg
# This represents a 2-D grid map
std_msgs/Header header
  builtin_interfaces/Time stamp
    int32 sec
    uint32 nanosec
  string frame_id

# MetaData for the map
MapMetaData info
  builtin_interfaces/Time map_load_time
    int32 sec
    uint32 nanosec
  float32 resolution
  uint32 width
  uint32 height
  geometry_msgs/Pose origin
    Point position
      float64 x
      float64 y
      float64 z
    Quaternion orientation
      float64 x 0
      float64 y 0
      float64 z 0
      float64 w 1

# The map data, in row-major order, starting with (0,0).
# Cell (1, 0) will be listed second, representing the next cell in the x direction.
# Cell (0, 1) will be at the index equal to info.width, followed by (1, 1).
# The values inside are application dependent, but frequently,
# 0 represents unoccupied, 1 represents definitely occupied, and
# -1 represents unknown.
int8[] data
```

---

## 3. 字段详解

### 3.1 `header`

```msg
stamp     # 时间戳（sec + nanosec）
frame_id  # 坐标系，泊车场景通常是 "map"
```

- `stamp`：栅格地图的生成时刻
- `frame_id`：地图所在坐标系。**planner 会用这个 frame 做坐标变换**——把车辆起点、目标点从 `map` 转到栅格坐标系。必须正确配置。

### 3.2 `info` —— 栅格元信息

| 字段 | 类型 | 含义 |
| --- | --- | --- |
| `map_load_time` | Time | 地图加载时间 |
| `resolution` | float32 | **分辨率**：每个格子代表的实际距离（米） |
| `width` | uint32 | 栅格宽度（格子数，x 方向） |
| `height` | uint32 | 栅格高度（格子数，y 方向） |
| `origin` | Pose | 格子 (0,0) 在**世界坐标系**中的位置和朝向 |

**覆盖范围计算**：

\[
\text{物理宽度} = \text{width} \times \text{resolution}, \quad
\text{物理高度} = \text{height} \times \text{resolution}
\]

例如 `resolution=0.3`、`width=200`、`height=200` → 60 m × 60 m 的区域。

### 3.3 `data` —— 栅格数据

**一维数组，按行优先（row-major）顺序存储**：

\[
\text{index} = y \times \text{width} + x
\]

```
        x=0   x=1   x=2 ...
y=0  [ data[0], data[1], data[2], ... ]
y=1  [ data[width], data[width+1], ... ]   ← 第 1 行从索引 width 开始
y=2  [ data[2*width], ... ]
```

- `Cell (1,0)` 是第 2 个元素（x 方向下一个格子）
- `Cell (0,1)` 在索引 `width` 处（下一行开头）

---

## 4. 格子占用值的含义

| 值 | 含义 | planner 处理 |
| --- | --- | --- |
| `0` | 空闲（可通行） | 可通行 |
| `1 ~ 99` | 占用程度 / 代价 | 可通行，值作为搜索代价（越小越倾向走） |
| `100` | 完全占用（障碍物） | **不可通行**（默认 `obstacle_threshold=100`） |
| `-1` | 未知 | **不可通行**（`cost < 0` 被当作障碍物） |

### Autoware 源码依据

`autoware_freespace_planning_algorithms/src/abstract_algorithm.cpp:110-125`：

```cpp
const int cost = costmap_.data[i];
if (cost < 0 || planner_common_param_.obstacle_threshold <= cost) {
  is_obstacle_table[i] = true;   // 标记为障碍物
}
```

即：
- `data[i] < 0`（未知）→ 障碍物
- `data[i] >= obstacle_threshold`（默认 100）→ 障碍物
- 其余 → 可通行

planner 把整个 `data` 转成 `is_obstacle_table`，再计算 EDT（欧氏距离变换）和碰撞索引表，用于 A* 搜索。

---

## 5. 具体示例

假设一条 occupancy grid：

```
resolution = 0.5 m
width = 100, height = 100       → 覆盖 50 m × 50 m
origin = (200, 200, 0)          → 栅格左下角在 map 坐标 (200, 200)
```

| 数据 | 含义 |
| --- | --- |
| `data[0] = 0` | 格子 (0,0) → map 坐标 (200, 200) 处空闲 |
| `data[5000] = 100` | 格子 (0, 50)（index = 50×100+0）→ map 坐标 (200, 225) 处是障碍物 |
| `data[i] = -1` | 对应格子未知，planner 当作障碍物 |

planner 会在这张 50×50 m 的栅格上搜索一条从当前位置到目标车位的无碰撞路径。

---

## 6. 自己生成 OccupancyGrid 时的检查清单

如果要绕过 `costmap_generator` 自己发布栅格地图，必须满足：

- [ ] `header.frame_id` 正确（通常 `map`），且 TF 中该 frame 与车辆位姿 frame 可互相转换
- [ ] `info.resolution` > 0，且与 `width`/`height` 匹配实际覆盖范围
- [ ] `info.origin` 正确设置（栅格左下角在世界坐标的位置和朝向）
- [ ] `data` 数组长度 = `width × height`
- [ ] 空闲区域填 `0`，障碍物填 `100`（或 ≥ `obstacle_threshold`），未知区域填 `-1`
- [ ] 栅格范围覆盖从当前位置到目标车位的整个区域
- [ ] 发布频率 ≥ planner 的 `update_rate`（默认 10 Hz）
- [ ] 不要和 `costmap_generator` 同时发布同一话题（多发布者冲突）

---

## 7. 在泊车链路中的位置

```
感知点云 + 感知 objects + 矢量地图
          ↓
   costmap_generator
          ↓
   /planning/scenario_planning/parking/costmap_generator/occupancy_grid
          ↓
   freespace_planner (setMap → is_obstacle_table → A* 搜索)
          ↓
   /planning/scenario_planning/parking/trajectory
```

本消息就是中间那条箭头——`costmap_generator` 的输出，`freespace_planner` 的输入。

