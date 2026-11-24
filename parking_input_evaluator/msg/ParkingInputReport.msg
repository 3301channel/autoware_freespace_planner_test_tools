# 泊车规划上游输入评估报告
std_msgs/Header header

bool is_input_ready          # 系统整体是否处于稳定可泊状态
bool can_plan_current_goal   # 本次 goal 是否可规划

uint8 worst_level            # 取值为 CheckItem 的级别常量
uint32 pass_count
uint32 warn_count
uint32 fail_count
string[] failure_reasons     # FAIL 项摘要，便于直接打印

CheckItem[] checks           # 23 项明细

# ---- 关键标量 ----
float64 grid_rate_hz
float64 coverage_ratio
float64 unknown_ratio
float64[4] grid_extent       # xmin, ymin, xmax, ymax
float64[4] parking_lot_extent
float64 goal_edt_margin
float64 goal_rect_conflicts

# ---- 车位统计 ----
uint32 total_spaces
uint32 parkable_spaces
uint32 out_of_grid_spaces
uint32 blocked_spaces
uint32 unreachable_spaces
ParkingSpaceStatus[] parking_spaces
