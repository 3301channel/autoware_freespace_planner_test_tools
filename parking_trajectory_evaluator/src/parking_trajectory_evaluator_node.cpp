// Copyright 2026
// Licensed under the Apache License, Version 2.0
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <string>
#include <vector>

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "diagnostic_msgs/msg/key_value.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

#include "autoware_internal_planning_msgs/msg/scenario.hpp"
#include "autoware_planning_msgs/msg/trajectory.hpp"

#include "parking_trajectory_evaluator/msg/metric_value.hpp"
#include "parking_trajectory_evaluator/msg/parking_trajectory_report.hpp"
#include "parking_trajectory_evaluator/srv/get_parking_report.hpp"

#include "parking_trajectory_evaluator/collision_checker.hpp"
#include "parking_trajectory_evaluator/param.hpp"
#include "parking_trajectory_evaluator/result.hpp"
#include "parking_trajectory_evaluator/scoring.hpp"
#include "parking_trajectory_evaluator/trajectory_metrics.hpp"
#include "parking_trajectory_evaluator/visualization.hpp"

namespace parking_trajectory_evaluator
{
namespace
{

double stampToSec(const builtin_interfaces::msg::Time & stamp)
{
  return static_cast<double>(stamp.sec) + static_cast<double>(stamp.nanosec) * 1e-9;
}

double yawFromQuaternion(const geometry_msgs::msg::Quaternion & q)
{
  return std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

}  // namespace

class ParkingTrajectoryEvaluatorNode : public rclcpp::Node
{
public:
  explicit ParkingTrajectoryEvaluatorNode(const rclcpp::NodeOptions & options)
  : rclcpp::Node("parking_trajectory_evaluator", options)
  {
    declareParameters();
    setupInterfaces();
    RCLCPP_INFO(
      get_logger(), "parking_trajectory_evaluator ready (mode=%s, occupancy=%s)",
      params_.evaluation_mode.c_str(), params_.enable_occupancy_check ? "on" : "off");
  }

private:
  // ---------------------------------------------------------------- params
  void declareParameters()
  {
    params_.vehicle.wheel_base = declare_parameter<double>("vehicle.wheel_base", 2.831);
    params_.vehicle.max_steer = declare_parameter<double>("vehicle.max_steer_angle", 0.70);
    params_.vehicle.length = declare_parameter<double>("vehicle.vehicle_length", 4.5136);
    params_.vehicle.width = declare_parameter<double>("vehicle.vehicle_width", 2.0068);
    params_.vehicle.rear_overhang = declare_parameter<double>("vehicle.rear_overhang", 1.1);

    params_.constraints.min_point_interval =
      declare_parameter<double>("constraints.min_point_interval", 0.01);
    params_.constraints.max_point_interval =
      declare_parameter<double>("constraints.max_point_interval", 2.0);
    params_.constraints.max_curvature =
      declare_parameter<double>("constraints.max_curvature", 0.398);
    params_.constraints.max_steer_rate =
      declare_parameter<double>("constraints.max_steer_angle_rate", 0.3);
    params_.constraints.max_velocity = declare_parameter<double>("constraints.max_velocity", 1.4);
    params_.constraints.terminal_velocity =
      declare_parameter<double>("constraints.terminal_velocity", 0.05);
    params_.constraints.min_points = declare_parameter<int>("constraints.min_points", 5);
    params_.constraints.max_delta_angle =
      declare_parameter<double>("constraints.max_relative_angle", 0.5);
    params_.constraints.occupancy_cost_threshold =
      declare_parameter<int>("constraints.occupancy_cost_threshold", 50);

    params_.comfort.max_lateral_acc = declare_parameter<double>("comfort.max_lateral_acc", 2.0);
    params_.comfort.max_lateral_jerk = declare_parameter<double>("comfort.max_lateral_jerk", 1.5);
    params_.comfort.max_longitudinal_acc =
      declare_parameter<double>("comfort.max_longitudinal_acc", 3.0);
    params_.comfort.max_longitudinal_jerk =
      declare_parameter<double>("comfort.max_longitudinal_jerk", 2.0);
    params_.comfort.max_switch_count = declare_parameter<int>("comfort.max_switch_count", 4);

    params_.goal.enabled = declare_parameter<bool>("goal.enabled", true);
    params_.goal.x = declare_parameter<double>("goal.x", 273.215);
    params_.goal.y = declare_parameter<double>("goal.y", 236.173);
    params_.goal.yaw = declare_parameter<double>("goal.yaw", 3.14159);
    params_.goal.tolerance_lat = declare_parameter<double>("goal.tolerance_lat", 0.3);
    params_.goal.tolerance_lon = declare_parameter<double>("goal.tolerance_lon", 0.5);
    params_.goal.tolerance_yaw = declare_parameter<double>("goal.tolerance_yaw", 0.1745);

    params_.scoring.min_score = declare_parameter<double>("scoring.min_score", 70.0);

    params_.evaluation_mode = declare_parameter<std::string>("evaluation_mode", "single");
    params_.enable_occupancy_check = declare_parameter<bool>("enable_occupancy_check", true);
    params_.enable_visualization = declare_parameter<bool>("enable_visualization", true);
    params_.publish_rate = declare_parameter<double>("publish_rate", 10.0);
    params_.require_parking_scenario = declare_parameter<bool>("require_parking_scenario", false);
  }

  void setupInterfaces()
  {
    const auto sensor_qos = rclcpp::SensorDataQoS();
    const auto reliable_qos = rclcpp::QoS(10).reliable();

    trajectory_sub_ = create_subscription<autoware_planning_msgs::msg::Trajectory>(
      "~/input/trajectory", reliable_qos,
      [this](const autoware_planning_msgs::msg::Trajectory::SharedPtr msg) { onTrajectory(msg); });

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "~/input/odometry", sensor_qos,
      [this](const nav_msgs::msg::Odometry::SharedPtr msg) { onOdometry(msg); });

    scenario_sub_ = create_subscription<autoware_internal_planning_msgs::msg::Scenario>(
      "~/input/scenario", reliable_qos,
      [this](const autoware_internal_planning_msgs::msg::Scenario::SharedPtr msg) {
        current_scenario_ = msg->current_scenario;
        has_scenario_ = true;
      });

    grid_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
      "~/input/occupancy_grid", sensor_qos,
      [this](const nav_msgs::msg::OccupancyGrid::SharedPtr msg) { onGrid(msg); });

    report_pub_ =
      create_publisher<msg::ParkingTrajectoryReport>("~/output/report", rclcpp::QoS(1));
    marker_pub_ =
      create_publisher<visualization_msgs::msg::MarkerArray>("~/output/marker", rclcpp::QoS(1));
    diag_pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>("/diagnostics", 10);

    service_ = create_service<srv::GetParkingReport>(
      "~/get_report",
      [this](
        const srv::GetParkingReport::Request::SharedPtr request,
        srv::GetParkingReport::Response::SharedPtr response) {
        response->report = toMessage(last_result_, last_trajectory_);
        if (!request->include_metrics) {
          response->report.metrics.clear();
        }
      });
  }

  // ---------------------------------------------------------------- callbacks
  void onTrajectory(const autoware_planning_msgs::msg::Trajectory::SharedPtr msg)
  {
    if (params_.require_parking_scenario && has_scenario_ && current_scenario_ != "Parking") {
      return;
    }

    TrajectoryData incoming;
    incoming.frame_id = msg->header.frame_id;
    incoming.stamp = stampToSec(msg->header.stamp);
    incoming.points.reserve(msg->points.size());
    for (const auto & p : msg->points) {
      PointData point;
      point.x = p.pose.position.x;
      point.y = p.pose.position.y;
      point.yaw = yawFromQuaternion(p.pose.orientation);
      point.v = static_cast<double>(p.longitudinal_velocity_mps);
      incoming.points.push_back(point);
    }

    latest_ = incoming;
    has_trajectory_ = true;

    const double now_sec = now().seconds();
    if (now_sec - last_eval_sec_ < 1.0 / std::max(params_.publish_rate, 1.0)) {
      return;
    }
    last_eval_sec_ = now_sec;
    evaluateAndPublish();
  }

  void onOdometry(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    odom_pose_ = msg->pose.pose;
    has_odom_ = true;
  }

  void onGrid(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
  {
    grid_.width = static_cast<int>(msg->info.width);
    grid_.height = static_cast<int>(msg->info.height);
    grid_.resolution = msg->info.resolution;
    grid_.origin_x = msg->info.origin.position.x;
    grid_.origin_y = msg->info.origin.position.y;
    grid_.frame_id = msg->header.frame_id;
    grid_.data.assign(msg->data.begin(), msg->data.end());
    has_grid_ = grid_.valid();
  }

  // ---------------------------------------------------------------- evaluation
  TrajectoryData collectForEvaluation(const TrajectoryData & incoming)
  {
    if (params_.evaluation_mode != "cumulative" || incoming.points.empty()) {
      return incoming;
    }

    if (!cumulative_.points.empty()) {
      const auto & last = cumulative_.points.back();
      const auto & first = incoming.points.front();
      if (std::hypot(first.x - last.x, first.y - last.y) > 1.0) {
        cumulative_.points.clear();
      }
    }
    for (const auto & p : incoming.points) {
      cumulative_.points.push_back(p);
    }
    constexpr size_t kMaxPoints = 3000;
    if (cumulative_.points.size() > kMaxPoints) {
      cumulative_.points.erase(
        cumulative_.points.begin(),
        cumulative_.points.begin() + static_cast<long>(cumulative_.points.size() - kMaxPoints));
    }
    cumulative_.frame_id = incoming.frame_id;
    cumulative_.stamp = incoming.stamp;
    return cumulative_;
  }

  void evaluateAndPublish()
  {
    const TrajectoryData trajectory = collectForEvaluation(latest_);
    const GridData * grid = (has_grid_ && params_.enable_occupancy_check) ? &grid_ : nullptr;

    EvaluationResult result = evaluateTrajectory(trajectory, grid, params_);

    // Informational: does the trajectory start where the vehicle is?
    if (has_odom_ && !trajectory.points.empty()) {
      const auto & first = trajectory.points.front();
      const double offset = std::hypot(
        first.x - odom_pose_.position.x, first.y - odom_pose_.position.y);
      MetricItem item;
      item.name = "start_offset";
      item.unit = "m";
      item.value = offset;
      item.threshold = 1.0;
      item.passed = offset <= 1.0;
      result.items.push_back(std::move(item));
    }

    last_result_ = result;
    last_trajectory_ = trajectory;

    report_pub_->publish(toMessage(result, trajectory));
    publishDiagnostics(result);
    if (params_.enable_visualization) {
      const auto markers = buildMarkers(
        result, trajectory, trajectory.frame_id.empty() ? "map" : trajectory.frame_id, now());
      marker_pub_->publish(markers);
    }
  }

  // ---------------------------------------------------------------- publishing
  msg::ParkingTrajectoryReport toMessage(
    const EvaluationResult & result, const TrajectoryData & trajectory) const
  {
    msg::ParkingTrajectoryReport message;
    message.header.stamp = now();
    message.header.frame_id = trajectory.frame_id.empty() ? "map" : trajectory.frame_id;

    message.is_control_ready = result.is_control_ready;
    message.failure_reasons = result.failure_reasons;
    message.total_score = result.score.total;

    for (const auto & item : result.items) {
      msg::MetricValue metric;
      metric.name = item.name;
      metric.value = item.value;
      metric.threshold = item.threshold;
      metric.passed = item.passed;
      metric.unit = item.unit;
      message.metrics.push_back(std::move(metric));
    }

    message.terminal_pose.position.x = result.terminal.x;
    message.terminal_pose.position.y = result.terminal.y;
    message.terminal_pose.orientation.z = std::sin(result.terminal.yaw / 2.0);
    message.terminal_pose.orientation.w = std::cos(result.terminal.yaw / 2.0);

    message.terminal_lat_error = result.lat_error;
    message.terminal_lon_error = result.lon_error;
    message.terminal_yaw_error = result.yaw_error;
    message.switch_count = result.metrics.switch_count;
    message.occupancy_evaluated = result.occupancy_evaluated;

    message.point_count = result.metrics.point_count;
    message.max_curvature = result.metrics.max_curvature;
    message.max_steer_angle = result.metrics.max_steer;
    message.max_steer_rate = result.metrics.max_steer_rate;
    message.max_abs_velocity = result.metrics.max_abs_velocity;
    message.terminal_velocity = result.metrics.terminal_velocity;
    message.max_lateral_acc = result.metrics.max_lateral_acc;
    message.max_lateral_jerk = result.metrics.max_lateral_jerk;
    message.max_longitudinal_acc = result.metrics.max_longitudinal_acc;
    message.max_longitudinal_jerk = result.metrics.max_longitudinal_jerk;
    message.min_point_interval = result.metrics.min_interval;
    message.max_point_interval = result.metrics.max_interval;
    message.max_delta_angle = result.metrics.max_delta_angle;
    message.center_collisions = result.collision.center_collisions;
    message.footprint_collisions = result.collision.footprint_collisions;
    message.trajectory_valid = result.metrics.valid;
    return message;
  }

  void publishDiagnostics(const EvaluationResult & result)
  {
    diagnostic_msgs::msg::DiagnosticArray array;
    array.header.stamp = now();

    diagnostic_msgs::msg::DiagnosticStatus status;
    status.name = "parking_trajectory_evaluator";
    status.hardware_id = "autoware";
    status.level = result.is_control_ready ? diagnostic_msgs::msg::DiagnosticStatus::OK
                                           : diagnostic_msgs::msg::DiagnosticStatus::ERROR;
    status.message = std::string("control_ready=") +
                     (result.is_control_ready ? "true" : "false") + " score=" +
                     std::to_string(static_cast<int>(result.score.total)) + " switches=" +
                     std::to_string(result.metrics.switch_count);

    auto addValue = [&status](const std::string & key, const std::string & value) {
      diagnostic_msgs::msg::KeyValue kv;
      kv.key = key;
      kv.value = value;
      status.values.push_back(std::move(kv));
    };

    for (const auto & reason : result.failure_reasons) {
      addValue("failure", reason);
    }
    addValue("max_curvature", std::to_string(result.metrics.max_curvature));
    addValue("max_velocity", std::to_string(result.metrics.max_abs_velocity));
    addValue("terminal_velocity", std::to_string(result.metrics.terminal_velocity));
    addValue("total_score", std::to_string(result.score.total));

    array.status.push_back(std::move(status));
    diag_pub_->publish(array);
  }

  // ---------------------------------------------------------------- members
  Params params_;

  TrajectoryData latest_;
  TrajectoryData cumulative_;
  TrajectoryData last_trajectory_;
  bool has_trajectory_{false};

  GridData grid_;
  bool has_grid_{false};

  geometry_msgs::msg::Pose odom_pose_;
  bool has_odom_{false};

  std::string current_scenario_;
  bool has_scenario_{false};

  double last_eval_sec_{0.0};
  EvaluationResult last_result_;

  rclcpp::Subscription<autoware_planning_msgs::msg::Trajectory>::SharedPtr trajectory_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<autoware_internal_planning_msgs::msg::Scenario>::SharedPtr scenario_sub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr grid_sub_;

  rclcpp::Publisher<msg::ParkingTrajectoryReport>::SharedPtr report_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diag_pub_;
  rclcpp::Service<srv::GetParkingReport>::SharedPtr service_;
};

}  // namespace parking_trajectory_evaluator

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<parking_trajectory_evaluator::ParkingTrajectoryEvaluatorNode>(
    rclcpp::NodeOptions());
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
