# 单个停车位的可泊状态
uint64 id
geometry_msgs/Point center
float64 heading          # rad，车位中心线主方向

# 状态常量
uint8 PARKABLE=0
uint8 OUT_OF_GRID=1
uint8 BLOCKED=2
uint8 UNREACHABLE=3
uint8 status

uint32 conflict_sample_count   # 车辆矩形采样冲突点数
float64 available_width        # 扣除障碍后的可用宽度估计 [m]
