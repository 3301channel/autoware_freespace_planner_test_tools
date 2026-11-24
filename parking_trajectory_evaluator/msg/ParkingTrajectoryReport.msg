# 泊车轨迹评估报告
std_msgs/Header header

bool is_control_ready          # 总体结论
string[] failure_reasons       # 硬约束失败原因
float64 total_score            # 软约束总分 0-100
MetricValue[] metrics          # 所有指标明细

geometry_msgs/Pose terminal_pose
float64 terminal_lat_error
float64 terminal_lon_error
float64 terminal_yaw_error
int32 switch_count
bool occupancy_evaluated

# ---- 便于快速查看的标量 ----
int32 point_count
float64 max_curvature
float64 max_steer_angle
float64 max_steer_rate
float64 max_abs_velocity
float64 terminal_velocity
float64 max_lateral_acc
float64 max_lateral_jerk
float64 max_longitudinal_acc
float64 max_longitudinal_jerk
float64 min_point_interval
float64 max_point_interval
float64 max_delta_angle
int32 center_collisions
int32 footprint_collisions
bool trajectory_valid
