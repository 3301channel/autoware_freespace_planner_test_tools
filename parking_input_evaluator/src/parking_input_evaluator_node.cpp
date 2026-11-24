// Copyright 2026
// Licensed under the Apache License, Version 2.0
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <deque>
#include <memory>
#include <string>
#include <vector>

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "diagnostic_msgs/msg/key_value.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "visualization_msgs/msg/marker_array.hpp"

#include "autoware_internal_planning_msgs/msg/scenario.hpp"
#include "autoware_planning_msgs/msg/lanelet_route.hpp"

#include "parking_input_evaluator/msg/check_item.hpp"
#include "parking_input_evaluator/msg/parking_input_report.hpp"
#include "parking_input_evaluator/msg/parking_space_status.hpp"
#include "parking_input_evaluator/srv/get_parking_input_report.hpp"

#include "parking_input_evaluator/connectivity.hpp"
#include "parking_input_evaluator/grid_utils.hpp"
#include "parking_input_evaluator/map_parser.hpp"
#include "parking_input_evaluator/param.hpp"
#include "parking_input_evaluator/pose_checker.hpp"
#include "parking_input_evaluator/result.hpp"
#include "parking_input_evaluator/visualization.hpp"

namespace parking_input_evaluator
{
namespace
{

double stampToSec(const builtin_interfaces::msg::Time & stamp)
{
  return static_cast<double>(stamp.sec) + static_cast<double>(stamp.nanosec) * 1e-9;
}

std::string formatDouble(double value, int precision = 2)
{
  if (!std::isfinite(value)) {
    return "n/a";
  }
  char buffer[64];
  std::snprintf(buffer, sizeof(buffer), "%.*f", precision, value);
  return std::string(buffer);
}

}  // namespace

class ParkingInputEvaluatorNode : public rclcpp::Node
{
public:
  explicit ParkingInputEvaluatorNode(const rclcpp::NodeOptions & options)
  : rclcpp::Node("parking_input_evaluator", options)
  {
    declareParameters();
    setupInterfaces();

    if (!params_.lanelet2_map_path.empty()) {
      map_ = loadMapGeometry(params_.lanelet2_map_path);
      map_loaded_ = map_.loaded;
      if (!map_loaded_) {
        RCLCPP_WARN(
          get_logger(), "Failed to load lanelet2 map '%s': %s",
          params_.lanelet2_map_path.c_str(), map_.error.c_str());
      } else {
        RCLCPP_INFO(
          get_logger(), "Loaded map: %zu parking spaces, lot found = %s",
          map_.spaces.size(), map_.has_lot ? "true" : "false");
      }
    }

    const auto period = std::chrono::duration<double>(1.0 / std::max(0.5, params_.update_rate));
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&ParkingInputEvaluatorNode::onTimer, this));
  }

  Report lastReport() const { return last_report_; }

private:
  // ---------------------------------------------------------------- params
  void declareParameters()
  {
    params_.vehicle.length = declare_parameter<double>("vehicle.length", 4.5136);
    params_.vehicle.width = declare_parameter<double>("vehicle.width", 2.0068);
    params_.vehicle.wheel_base = declare_parameter<double>("vehicle.wheel_base", 2.831);
    params_.vehicle.max_steer = declare_parameter<double>("vehicle.max_steer_angle", 0.70);
    params_.vehicle.margin = declare_parameter<double>("vehicle.vehicle_shape_margin", 0.05);

    params_.grid.expected_frame = declare_parameter<std::string>("grid.expected_frame", "map");
    params_.grid.expected_resolution = declare_parameter<double>("grid.expected_resolution", 0.3);
    params_.grid.resolution_tolerance =
      declare_parameter<double>("grid.resolution_tolerance", 0.001);
    params_.grid.expected_rate_hz = declare_parameter<double>("grid.expected_rate_hz", 10.0);
    params_.grid.rate_tolerance = declare_parameter<double>("grid.rate_tolerance", 0.2);
    params_.grid.obstacle_threshold = declare_parameter<int>("grid.obstacle_threshold", 100);
    params_.grid.coverage_fail = declare_parameter<double>("grid.coverage_fail", 0.75);
    params_.grid.coverage_pass = declare_parameter<double>("grid.coverage_pass", 0.95);
    params_.grid.unknown_fail = declare_parameter<double>("grid.unknown_fail", 0.20);
    params_.grid.unknown_pass = declare_parameter<double>("grid.unknown_pass", 0.05);

    params_.goal.angle_tolerance_pass_deg =
      declare_parameter<double>("goal.angle_tolerance_pass_deg", 6.0);
    params_.goal.angle_tolerance_warn_deg =
      declare_parameter<double>("goal.angle_tolerance_warn_deg", 18.0);
    params_.goal.require_in_parking_space =
      declare_parameter<bool>("goal.require_in_parking_space", true);
    params_.goal.rect_samples_long = declare_parameter<int>("goal.rect_samples_long", 23);
    params_.goal.rect_samples_lat = declare_parameter<int>("goal.rect_samples_lat", 13);

    params_.start.route_gap_fail = declare_parameter<double>("start.route_gap_fail", 3.0);
    params_.start.route_gap_pass = declare_parameter<double>("start.route_gap_pass", 1.0);
    params_.start.odom_rate_min = declare_parameter<double>("start.odom_rate_min", 5.0);
    params_.start.odom_rate_max = declare_parameter<double>("start.odom_rate_max", 20.0);

    params_.scenario.required = declare_parameter<std::string>("scenario.required", "Parking");
    params_.scenario.ratio_fail = declare_parameter<double>("scenario.ratio_fail", 0.30);
    params_.scenario.ratio_pass = declare_parameter<double>("scenario.ratio_pass", 0.90);
    params_.scenario.enabled = declare_parameter<bool>("scenario.enabled", true);

    params_.connectivity.edt_margin_pass =
      declare_parameter<double>("connectivity.edt_margin_pass", 0.10);
    params_.connectivity.edt_margin_warn =
      declare_parameter<double>("connectivity.edt_margin_warn", -0.05);
    params_.connectivity.strict_rect_connectivity =
      declare_parameter<bool>("connectivity.strict_rect_connectivity", false);
    params_.connectivity.neighbors = declare_parameter<int>("connectivity.connectivity_neighbors", 8);

    params_.spaces.enabled = declare_parameter<bool>("parking_spaces.enabled", true);
    params_.spaces.check_mode =
      declare_parameter<std::string>("parking_spaces.check_mode", "center");
    params_.spaces.multi_pose_count = declare_parameter<int>("parking_spaces.multi_pose_count", 3);
    params_.spaces.width_margin_fail =
      declare_parameter<double>("parking_spaces.width_margin_fail", 0.10);
    params_.spaces.width_margin_pass =
      declare_parameter<double>("parking_spaces.width_margin_pass", 0.20);
    params_.spaces.length_margin_fail =
      declare_parameter<double>("parking_spaces.length_margin_fail", 0.05);
    params_.spaces.length_margin_pass =
      declare_parameter<double>("parking_spaces.length_margin_pass", 0.20);
    params_.spaces.parkable_ratio_fail =
      declare_parameter<double>("parking_spaces.parkable_ratio_fail", 0.50);
    params_.spaces.parkable_ratio_pass =
      declare_parameter<double>("parking_spaces.parkable_ratio_pass", 0.90);

    params_.tf.global_frame = declare_parameter<std::string>("tf.global_frame", "map");
    params_.tf.base_frame = declare_parameter<std::string>("tf.base_frame", "base_link");
    params_.tf.timeout_sec = declare_parameter<double>("tf.timeout_sec", 0.1);
    params_.tf.missing_is_fail = declare_parameter<bool>("tf.missing_is_fail", false);

    params_.lanelet2_map_path = declare_parameter<std::string>("lanelet2_map_path", "");
    params_.update_rate = declare_parameter<double>("update_rate", 5.0);
    params_.debounce_frames = declare_parameter<int>("debounce_frames", 3);
    params_.publish_marker = declare_parameter<bool>("publish_marker", true);
    params_.enable_service = declare_parameter<bool>("enable_service", true);
  }

  void setupInterfaces()
  {
    const auto sensor_qos = rclcpp::SensorDataQoS();
    // Volatile reliable is the most permissive combination: it also accepts
    // publishers that offer transient_local, which rosbag2 and the planner do.
    const auto latched_qos = rclcpp::QoS(10).reliable();

    grid_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
      "~/input/occupancy_grid", sensor_qos,
      [this](const nav_msgs::msg::OccupancyGrid::SharedPtr msg) { onGrid(msg); });

    route_sub_ = create_subscription<autoware_planning_msgs::msg::LaneletRoute>(
      "~/input/route", latched_qos,
      [this](const autoware_planning_msgs::msg::LaneletRoute::SharedPtr msg) { onRoute(msg); });

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "~/input/odometry", sensor_qos,
      [this](const nav_msgs::msg::Odometry::SharedPtr msg) { onOdom(msg); });

    scenario_sub_ = create_subscription<autoware_internal_planning_msgs::msg::Scenario>(
      "~/input/scenario", latched_qos,
      [this](const autoware_internal_planning_msgs::msg::Scenario::SharedPtr msg) {
        onScenario(msg);
      });

    report_pub_ = create_publisher<msg::ParkingInputReport>("~/output/report", rclcpp::QoS(1));
    marker_pub_ =
      create_publisher<visualization_msgs::msg::MarkerArray>("~/output/marker", rclcpp::QoS(1));
    diag_pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>("/diagnostics", 10);

    if (params_.enable_service) {
      service_ = create_service<srv::GetParkingInputReport>(
        "~/get_report",
        [this](
          const srv::GetParkingInputReport::Request::SharedPtr request,
          srv::GetParkingInputReport::Response::SharedPtr response) {
          response->report = toMessage(last_report_);
          if (!request->include_space_details) {
            response->report.parking_spaces.clear();
          }
        });
    }

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  }

  // ---------------------------------------------------------------- callbacks
  void onGrid(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
  {
    grid_.info.width = static_cast<int>(msg->info.width);
    grid_.info.height = static_cast<int>(msg->info.height);
    grid_.info.resolution = msg->info.resolution;
    grid_.info.origin_x = msg->info.origin.position.x;
    grid_.info.origin_y = msg->info.origin.position.y;
    grid_.info.frame_id = msg->header.frame_id;
    grid_.data.assign(msg->data.begin(), msg->data.end());
    grid_origin_identity_ =
      std::fabs(msg->info.origin.orientation.z) < 1e-3 &&
      std::fabs(msg->info.origin.orientation.w - 1.0) < 1e-3;

    if (grid_.valid()) {
      ++grid_count_;
      has_grid_ = true;
      // Use the local receipt time so the measured rate reflects how often a
      // fresh grid actually becomes available, not the nominal stamp interval.
      const double receipt = now().seconds();
      grid_stamps_.push_back(receipt > 0.0 ? receipt : stampToSec(msg->header.stamp));
      while (grid_stamps_.size() > 200) {
        grid_stamps_.pop_front();
      }
    }
  }

  void onRoute(const autoware_planning_msgs::msg::LaneletRoute::SharedPtr msg)
  {
    route_goal_ = msg->goal_pose;
    route_start_ = msg->start_pose;
    // Snapshot the vehicle pose at the moment the route arrives. C4 must
    // compare the route origin against where the vehicle actually was when the
    // goal was set, not against the moving pose at evaluation time.
    if (has_odom_) {
      odom_at_route_ = odom_.pose.pose;
      has_odom_at_route_ = true;
    }
    has_route_ = true;
  }

  void onOdom(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    odom_ = *msg;
    has_odom_ = true;
    const double receipt = now().seconds();
    odom_stamps_.push_back(receipt > 0.0 ? receipt : stampToSec(msg->header.stamp));
    while (odom_stamps_.size() > 200) {
      odom_stamps_.pop_front();
    }
  }

  void onScenario(const autoware_internal_planning_msgs::msg::Scenario::SharedPtr msg)
  {
    scenario_history_.push_back(msg->current_scenario);
    while (scenario_history_.size() > 200) {
      scenario_history_.pop_front();
    }
  }

  // ---------------------------------------------------------------- timer
  void onTimer()
  {
    if (!has_grid_ || !has_route_ || !has_odom_) {
      publishWaitingDiagnostics();
      return;
    }

    Report report = evaluate();
    last_report_ = report;

    if (report.worst_level == last_published_level_) {
      ++stable_count_;
    } else {
      stable_count_ = 1;
      last_published_level_ = report.worst_level;
    }

    const bool urgent = report.worst_level == Level::FAIL;
    if (urgent || stable_count_ >= std::max(1, params_.debounce_frames)) {
      publishReport(report);
      publishDiagnostics(report);
      if (params_.publish_marker) {
        publishMarkers(report);
      }
    }
  }

  void publishWaitingDiagnostics()
  {
    diagnostic_msgs::msg::DiagnosticArray array;
    array.header.stamp = now();
    diagnostic_msgs::msg::DiagnosticStatus status;
    status.name = "parking_input_evaluator";
    status.hardware_id = "autoware";
    status.level = diagnostic_msgs::msg::DiagnosticStatus::STALE;
    status.message = "waiting for inputs";
    if (!has_grid_) {
      status.message += " [grid]";
    }
    if (!has_route_) {
      status.message += " [route]";
    }
    if (!has_odom_) {
      status.message += " [odom]";
    }
    array.status.push_back(std::move(status));
    diag_pub_->publish(array);
  }

  // ---------------------------------------------------------------- evaluation
  Report evaluate()
  {
    Report report;
    const double col_len = params_.vehicle.collisionLength();
    const double col_wid = params_.vehicle.collisionWidth();
    const double half_wid = col_wid / 2.0;
    const int threshold = params_.grid.obstacle_threshold;

    const double gx0 = grid_.info.origin_x;
    const double gy0 = grid_.info.origin_y;
    const double gx1 = gx0 + grid_.info.width * grid_.info.resolution;
    const double gy1 = gy0 + grid_.info.height * grid_.info.resolution;
    report.grid_extent[0] = gx0;
    report.grid_extent[1] = gy0;
    report.grid_extent[2] = gx1;
    report.grid_extent[3] = gy1;

    // ---------------- A1 grid rate ----------------
    double grid_hz = 0.0;
    if (grid_stamps_.size() >= 2) {
      const double span = grid_stamps_.back() - grid_stamps_.front();
      if (span > 1e-6) {
        grid_hz = static_cast<double>(grid_stamps_.size() - 1) / span;
      }
    }
    report.grid_rate_hz = grid_hz;

    const double low_rate = params_.grid.expected_rate_hz * (1.0 - params_.grid.rate_tolerance);
    const double high_rate = params_.grid.expected_rate_hz * (1.0 + params_.grid.rate_tolerance);
    report.addCheck(
      "A1", "栅格发布频率",
      classify(grid_hz >= low_rate && grid_hz <= high_rate, grid_hz > 0.0),
      formatDouble(grid_hz, 2) + " Hz (" + std::to_string(grid_count_) + " msgs), expected " +
        formatDouble(params_.grid.expected_rate_hz, 1) + " Hz",
      grid_hz >= low_rate ? std::string()
                          : "costmap source is slower than the planner rate; check its load");

    // ---------------- A2 metadata ----------------
    const bool frame_ok = grid_.info.frame_id == params_.grid.expected_frame;
    const bool resolution_ok =
      std::fabs(grid_.info.resolution - params_.grid.expected_resolution) <=
      params_.grid.resolution_tolerance;
    report.addCheck(
      "A2", "栅格元数据", classify(frame_ok && resolution_ok && grid_origin_identity_, true),
      "frame_id=" + grid_.info.frame_id + ", resolution=" + formatDouble(grid_.info.resolution, 3) +
        " m, size=" + std::to_string(grid_.info.width) + "x" +
        std::to_string(grid_.info.height),
      frame_ok ? std::string() : "frame_id does not match the planner assumption");

    // ---------------- A3 coverage ----------------
    double coverage = 0.0;
    if (map_.loaded && map_.has_lot) {
      report.lot_extent[0] = map_.lot_min_x;
      report.lot_extent[1] = map_.lot_min_y;
      report.lot_extent[2] = map_.lot_max_x;
      report.lot_extent[3] = map_.lot_max_y;

      const double ix0 = std::max(gx0, map_.lot_min_x);
      const double iy0 = std::max(gy0, map_.lot_min_y);
      const double ix1 = std::min(gx1, map_.lot_max_x);
      const double iy1 = std::min(gy1, map_.lot_max_y);
      const double inter = std::max(0.0, ix1 - ix0) * std::max(0.0, iy1 - iy0);
      const double lot_area = std::max(
        1e-6, (map_.lot_max_x - map_.lot_min_x) * (map_.lot_max_y - map_.lot_min_y));
      coverage = inter / lot_area;
      report.coverage_ratio = coverage;

      std::string gaps;
      if (gx1 < map_.lot_max_x) {
        gaps += " right missing " + formatDouble(map_.lot_max_x - gx1, 1) + " m";
      }
      if (gx0 > map_.lot_min_x) {
        gaps += " left missing " + formatDouble(gx0 - map_.lot_min_x, 1) + " m";
      }
      if (gy1 < map_.lot_max_y) {
        gaps += " top missing " + formatDouble(map_.lot_max_y - gy1, 1) + " m";
      }
      if (gy0 > map_.lot_min_y) {
        gaps += " bottom missing " + formatDouble(gy0 - map_.lot_min_y, 1) + " m";
      }
      report.addCheck(
        "A3", "停车场覆盖率",
        classify(coverage >= params_.grid.coverage_pass, coverage >= params_.grid.coverage_fail),
        formatDouble(coverage * 100.0, 1) + "%" + (gaps.empty() ? " (full)" : "," + gaps),
        coverage >= params_.grid.coverage_pass ? std::string()
                                               : "increase grid_length_x/y or centre the grid");
    } else {
      report.addCheck(
        "A3", "停车场覆盖率", Level::WARN, "parking_lot geometry unavailable",
        "check lanelet2_map_path and the map contents");
    }

    // ---------------- A4 value distribution ----------------
    const CostStats stats = computeCostStats(grid_, threshold);
    report.unknown_ratio = stats.unknownRatio();
    report.addCheck(
      "A4", "栅格未知区域",
      classify(
        stats.unknownRatio() < params_.grid.unknown_pass,
        stats.unknownRatio() < params_.grid.unknown_fail),
      "unknown " + formatDouble(stats.unknownRatio() * 100.0, 1) + "%, occupied " +
        formatDouble(stats.occupiedRatio() * 100.0, 1) + "%, free " +
        formatDouble(stats.freeRatio() * 100.0, 1) + "%",
      stats.unknownRatio() < params_.grid.unknown_pass
        ? std::string()
        : "unknown(-1) is treated as an obstacle by A*");

    // ---------------- B route / goal ----------------
    bool goal_ok = false;
    if (!has_route_) {
      report.addCheck("B1", "route 输入", Level::FAIL, "no route received");
    } else {
      const double gyaw = yawFromQuaternion(
        route_goal_.orientation.x, route_goal_.orientation.y,
        route_goal_.orientation.z, route_goal_.orientation.w);
      report.addCheck(
        "B1", "route 输入", Level::PASS,
        "goal=(" + formatDouble(route_goal_.position.x) + ", " +
          formatDouble(route_goal_.position.y) + ") yaw=" +
          formatDouble(gyaw * 180.0 / M_PI, 1) + " deg");

      const CellIndex goal_cell = worldToCell(route_goal_.position.x, route_goal_.position.y, grid_.info);
      report.addCheck(
        "B2", "goal 在栅格内", classify(goal_cell.inside, goal_cell.inside),
        "cell=(" + std::to_string(goal_cell.ix) + "," + std::to_string(goal_cell.iy) + ")",
        goal_cell.inside ? std::string() : "goal outside the grid; A* cannot reach it");

      bool goal_free = false;
      if (goal_cell.inside) {
        const int cost = static_cast<int>(costAt(grid_, goal_cell.ix, goal_cell.iy));
        goal_free = !isObstacle(static_cast<int8_t>(cost), threshold);
        report.addCheck(
          "B3", "goal 单元格可通行", classify(goal_free, goal_free),
          "cost=" + std::to_string(cost),
          goal_free ? std::string() : "goal cell is occupied or unknown");
      } else {
        report.addCheck("B3", "goal 单元格可通行", Level::FAIL, "goal outside grid");
      }

      const RectCheck goal_rect = sampleRect(
        grid_, route_goal_.position.x, route_goal_.position.y, gyaw, col_len, col_wid,
        params_.goal.rect_samples_long, params_.goal.rect_samples_lat, threshold);
      report.goal_rect_conflicts = static_cast<double>(goal_rect.conflicts);
      report.addCheck(
        "B4", "goal 处可容纳车辆矩形", classify(goal_rect.ok, goal_rect.ok),
        "rect " + formatDouble(col_len, 3) + "x" + formatDouble(col_wid, 3) + " m, conflicts=" +
          std::to_string(goal_rect.conflicts),
        goal_rect.ok ? std::string() : "vehicle footprint overlaps obstacles at the goal");

      PoseCheckResult goal_pose_check;
      if (map_.loaded) {
        goal_pose_check = checkPose(
          grid_, route_goal_.position.x, route_goal_.position.y, gyaw, col_len, col_wid, threshold,
          params_.goal.rect_samples_long, params_.goal.rect_samples_lat, &map_, false);
        const double diff = goal_pose_check.heading_diff_deg;
        const bool in_band = goal_pose_check.in_space_band;
        report.addCheck(
          "B5", "目标朝向与车位一致性",
          classify(
            (!params_.goal.require_in_parking_space || in_band) &&
              diff <= params_.goal.angle_tolerance_pass_deg,
            (!params_.goal.require_in_parking_space || in_band) &&
              diff <= params_.goal.angle_tolerance_warn_deg),
          "nearest bay " + formatDouble(goal_pose_check.nearest_space_distance) +
            " m, in_band=" + (in_band ? "true" : "false") + ", heading diff " +
            formatDouble(diff, 1) + " deg",
          diff <= params_.goal.angle_tolerance_pass_deg
            ? std::string()
            : "goal heading or position does not match a parking bay");
      } else {
        report.addCheck("B5", "目标朝向与车位一致性", Level::WARN, "map not loaded");
      }

      goal_ok = goal_cell.inside && goal_free && goal_rect.ok &&
                (goal_pose_check.heading_diff_deg <= params_.goal.angle_tolerance_warn_deg ||
                 !map_.loaded);
    }

    // ---------------- C odometry / start ----------------
    double odom_hz = 0.0;
    if (odom_stamps_.size() >= 2) {
      const double span = odom_stamps_.back() - odom_stamps_.front();
      if (span > 1e-6) {
        odom_hz = static_cast<double>(odom_stamps_.size() - 1) / span;
      }
    }
    report.addCheck(
      "C1", "odometry 频率",
      classify(
        odom_hz >= params_.start.odom_rate_min && odom_hz <= params_.start.odom_rate_max, odom_hz > 0.0),
      formatDouble(odom_hz, 2) + " Hz");

    const auto & start = odom_.pose.pose;
    const double start_yaw = yawFromQuaternion(
      start.orientation.x, start.orientation.y, start.orientation.z, start.orientation.w);
    const CellIndex start_cell = worldToCell(start.position.x, start.position.y, grid_.info);
    report.addCheck(
      "C2", "start 在栅格内", classify(start_cell.inside, start_cell.inside),
      "cell=(" + std::to_string(start_cell.ix) + "," + std::to_string(start_cell.iy) + ")",
      start_cell.inside ? std::string() : "vehicle outside the grid");

    RectCheck start_rect;
    if (start_cell.inside) {
      start_rect = sampleRect(
        grid_, start.position.x, start.position.y, start_yaw, col_len, col_wid,
        params_.goal.rect_samples_long, params_.goal.rect_samples_lat, threshold);
    } else {
      start_rect.ok = false;
    }
    report.addCheck(
      "C3", "start 处可容纳车辆矩形", classify(start_rect.ok, start_rect.ok),
      "conflicts=" + std::to_string(start_rect.conflicts),
      start_rect.ok ? std::string() : "the start pose itself collides");

    if (has_route_) {
      const auto & route_pose = has_odom_at_route_ ? odom_at_route_ : start;
      const double gap = std::hypot(
        route_pose.position.x - route_start_.position.x,
        route_pose.position.y - route_start_.position.y);
      report.addCheck(
        "C4", "odom 与 route.start 一致性",
        classify(gap < params_.start.route_gap_pass, gap < params_.start.route_gap_fail),
        "gap " + formatDouble(gap) + " m",
        gap < params_.start.route_gap_pass ? std::string()
                                           : "set the goal while the vehicle is stationary");
    }

    // ---------------- D scenario ----------------
    if (params_.scenario.enabled) {
      if (scenario_history_.empty()) {
        report.addCheck("D1", "scenario 处于 Parking", Level::WARN, "no scenario received");
      } else {
        size_t parking = 0;
        for (const auto & s : scenario_history_) {
          if (s == params_.scenario.required) {
            ++parking;
          }
        }
        const double ratio =
          static_cast<double>(parking) / static_cast<double>(scenario_history_.size());
        report.addCheck(
          "D1", "scenario 处于 Parking",
          classify(ratio > params_.scenario.ratio_pass, ratio > params_.scenario.ratio_fail),
          formatDouble(ratio * 100.0, 1) + "% of recent messages",
          ratio > params_.scenario.ratio_pass ? std::string()
                                              : "planner is not activated outside Parking");
      }
    }

    // ---------------- E TF ----------------
    bool tf_ok = false;
    try {
      tf_ok = tf_buffer_->canTransform(
        params_.tf.global_frame, params_.tf.base_frame, tf2::TimePointZero,
        tf2::durationFromSec(params_.tf.timeout_sec));
    } catch (const std::exception &) {
      tf_ok = false;
    }
    report.addCheck(
      "E1", "map -> base_link TF", classify(tf_ok, !params_.tf.missing_is_fail),
      tf_ok ? "available" : "missing",
      tf_ok ? std::string() : "the transform chain map -> base_link is required");

    // ---------------- F connectivity ----------------
    const GridPoint start_point{start_cell.ix, start_cell.iy, start_cell.inside};
    const CellIndex goal_cell = worldToCell(route_goal_.position.x, route_goal_.position.y, grid_.info);
    const GridPoint goal_point{goal_cell.ix, goal_cell.iy, goal_cell.inside};

    ConnectivityResult connectivity;
    if (start_cell.inside && goal_cell.inside) {
      connectivity = analyzeConnectivity(
        grid_, threshold, half_wid, start_point, goal_point, params_.connectivity.neighbors);

      report.addCheck(
        "F1", "start-goal 拓扑连通(free 区)", classify(connectivity.free_connected, true),
        connectivity.free_connected ? "same component" : "different components",
        connectivity.free_connected ? std::string() : "grid is split by obstacles");

      report.addCheck(
        "F2", "A* 可行连通(EDT >= half width)", classify(connectivity.passable_connected, true),
        "passable " + formatDouble(
                        100.0 * connectivity.passable_count /
                          std::max(1, grid_.info.width * grid_.info.height),
                        1) +
          "%, start=" + (connectivity.start_passable ? "ok" : "no") + ", goal=" +
          (connectivity.goal_passable ? "ok" : "no"),
        connectivity.passable_connected
          ? std::string()
          : "no A* path exists with the EDT constraint; bays are too narrow");

      if (goal_cell.inside) {
        const double goal_edt = edtAt(connectivity, grid_, goal_cell.ix, goal_cell.iy);
        const double margin = goal_edt - half_wid;
        report.goal_edt_margin = margin;
        report.addCheck(
          "F3", "goal 处 EDT 余量",
          classify(
            margin >= params_.connectivity.edt_margin_pass,
            margin >= params_.connectivity.edt_margin_warn),
          "edt " + formatDouble(goal_edt) + " m, half width " + formatDouble(half_wid) +
            " m, margin " + formatDouble(margin) + " m",
          margin >= params_.connectivity.edt_margin_pass
            ? std::string()
            : "very few discrete headings can reach the goal");
      }
    } else {
      report.addCheck("F1", "start-goal 拓扑连通(free 区)", Level::FAIL, "start or goal outside grid");
      report.addCheck("F2", "A* 可行连通(EDT >= half width)", Level::FAIL, "start or goal outside grid");
    }

    // ---------------- F4 per-bay parkability ----------------
    if (params_.spaces.enabled && map_.loaded && !map_.spaces.empty() && start_cell.inside) {
      uint32_t parkable = 0;
      uint32_t out_of_grid = 0;
      uint32_t blocked = 0;
      uint32_t unreachable = 0;

      for (const auto & space : map_.spaces) {
        SpaceResult result;
        result.id = space.id;
        const Point2D center = space.center();
        result.cx = center.x;
        result.cy = center.y;
        result.heading = space.heading();
        result.available_width = space.width;

        const CellIndex cell = worldToCell(center.x, center.y, grid_.info);
        if (!cell.inside) {
          result.status = SpaceStatus::OUT_OF_GRID;
          ++out_of_grid;
        } else {
          const RectCheck rect = sampleRect(
            grid_, center.x, center.y, result.heading, col_len, col_wid,
            params_.goal.rect_samples_long, params_.goal.rect_samples_lat, threshold);
          result.conflict_samples = static_cast<uint32_t>(rect.conflicts);

          if (!rect.ok) {
            result.status = SpaceStatus::BLOCKED;
            ++blocked;
          } else {
            const size_t index = static_cast<size_t>(cell.iy) * grid_.info.width + cell.ix;
            const bool same_component =
              index < connectivity.passable_labels.size() &&
              connectivity.passable_labels[index] != 0 &&
              connectivity.passable_labels[index] == connectivity.start_label;
            if (!same_component) {
              result.status = SpaceStatus::UNREACHABLE;
              ++unreachable;
            } else {
              result.status = SpaceStatus::PARKABLE;
              ++parkable;
            }
          }
        }
        report.spaces.push_back(result);
      }

      report.total_spaces = static_cast<uint32_t>(map_.spaces.size());
      report.parkable_spaces = parkable;
      report.out_of_grid_spaces = out_of_grid;
      report.blocked_spaces = blocked;
      report.unreachable_spaces = unreachable;

      const double ratio =
        static_cast<double>(parkable) / static_cast<double>(std::max<size_t>(1, map_.spaces.size()));
      report.addCheck(
        "F4", "车位可停泊比例",
        classify(ratio >= params_.spaces.parkable_ratio_pass, ratio >= params_.spaces.parkable_ratio_fail),
        std::to_string(parkable) + "/" + std::to_string(map_.spaces.size()) + " (" +
          formatDouble(ratio * 100.0, 1) + "%), out_of_grid=" + std::to_string(out_of_grid) +
          ", blocked=" + std::to_string(blocked) + ", unreachable=" +
          std::to_string(unreachable),
        ratio >= params_.spaces.parkable_ratio_pass
          ? std::string()
          : "a bay that is not parkable fails the whole parking task");
    } else if (params_.spaces.enabled) {
      report.addCheck("F4", "车位可停泊比例", Level::WARN, "no parking spaces in the map");
    }

    // ---------------- G geometry ----------------
    if (map_.loaded && !map_.spaces.empty()) {
      double min_width = map_.spaces.front().width;
      double min_length = map_.spaces.front().length();
      for (const auto & space : map_.spaces) {
        min_width = std::min(min_width, space.width);
        min_length = std::min(min_length, space.length());
      }

      const double width_margin = (min_width - col_wid) / 2.0;
      report.addCheck(
        "G1", "车位宽度 vs 碰撞形状",
        classify(
          width_margin >= params_.spaces.width_margin_pass,
          width_margin >= params_.spaces.width_margin_fail),
        "bay " + formatDouble(min_width) + " m, collision width " + formatDouble(col_wid, 3) +
          " m, side margin " + formatDouble(width_margin * 100.0, 1) + " cm",
        width_margin >= params_.spaces.width_margin_pass
          ? std::string()
          : "tight bays produce false collisions under heading discretisation");

      const double length_margin = (min_length - col_len) / 2.0;
      report.addCheck(
        "G2", "车位长度 vs 碰撞形状",
        classify(
          length_margin >= params_.spaces.length_margin_pass,
          length_margin >= params_.spaces.length_margin_fail),
        "bay " + formatDouble(min_length) + " m, collision length " + formatDouble(col_len, 3) +
          " m, side margin " + formatDouble(length_margin * 100.0, 1) + " cm");
    }

    const double theta_step = 360.0 / 120.0;
    const double steps = params_.goal.angle_tolerance_pass_deg / theta_step;
    report.addCheck(
      "G3", "朝向容差的离散化", classify(steps >= 1.0, true),
      "theta_size=120 -> " + formatDouble(theta_step, 0) + " deg/step, tolerance +/-" +
        formatDouble(params_.goal.angle_tolerance_pass_deg, 0) + " deg = " +
        formatDouble(steps, 1) + " steps",
      steps >= 1.0 ? std::string() : "tolerance smaller than one step");

    if (map_.loaded && map_.has_lot) {
      const double diagonal = map_.lotDiagonal();
      const double side = grid_.info.width * grid_.info.resolution;
      report.addCheck(
        "G4", "栅格边长 vs 停车场对角线", classify(side >= diagonal, side >= 0.85 * diagonal),
        "grid " + formatDouble(side, 1) + " m, lot diagonal " + formatDouble(diagonal, 1) + " m",
        side >= diagonal ? std::string() : "a grid smaller than the lot always has blind areas");
    }

    // ---------------- verdict ----------------
    const bool current_goal_ok =
      goal_ok && (!connectivity.start_inside || connectivity.free_connected) &&
      (!connectivity.start_inside || connectivity.passable_connected);
    report.finalize(current_goal_ok);
    return report;
  }

  // ---------------------------------------------------------------- publishing
  msg::ParkingInputReport toMessage(const Report & report) const
  {
    msg::ParkingInputReport message;
    message.header.stamp = now();
    message.header.frame_id = grid_.info.frame_id.empty() ? "map" : grid_.info.frame_id;

    message.is_input_ready = report.is_input_ready;
    message.can_plan_current_goal = report.can_plan_current_goal;
    message.worst_level = static_cast<uint8_t>(report.worst_level);
    message.pass_count = report.pass_count;
    message.warn_count = report.warn_count;
    message.fail_count = report.fail_count;
    message.failure_reasons = report.failure_reasons;

    for (const auto & check : report.checks) {
      msg::CheckItem item;
      item.id = check.id;
      item.title = check.title;
      item.level = static_cast<uint8_t>(check.level);
      item.value = check.value;
      item.threshold_pass = check.threshold_pass;
      item.threshold_warn = check.threshold_warn;
      item.unit = check.unit;
      item.detail = check.detail;
      item.advice = check.advice;
      message.checks.push_back(std::move(item));
    }

    message.grid_rate_hz = report.grid_rate_hz;
    message.coverage_ratio = report.coverage_ratio;
    message.unknown_ratio = report.unknown_ratio;
    message.grid_extent = {
      report.grid_extent[0], report.grid_extent[1], report.grid_extent[2], report.grid_extent[3]};
    message.parking_lot_extent = {
      report.lot_extent[0], report.lot_extent[1], report.lot_extent[2], report.lot_extent[3]};
    message.goal_edt_margin = report.goal_edt_margin;
    message.goal_rect_conflicts = report.goal_rect_conflicts;

    message.total_spaces = report.total_spaces;
    message.parkable_spaces = report.parkable_spaces;
    message.out_of_grid_spaces = report.out_of_grid_spaces;
    message.blocked_spaces = report.blocked_spaces;
    message.unreachable_spaces = report.unreachable_spaces;

    for (const auto & space : report.spaces) {
      msg::ParkingSpaceStatus status;
      status.id = space.id;
      status.center.x = space.cx;
      status.center.y = space.cy;
      status.heading = space.heading;
      status.status = static_cast<uint8_t>(space.status);
      status.conflict_sample_count = space.conflict_samples;
      status.available_width = space.available_width;
      message.parking_spaces.push_back(std::move(status));
    }
    return message;
  }

  void publishReport(const Report & report) { report_pub_->publish(toMessage(report)); }

  void publishDiagnostics(const Report & report)
  {
    diagnostic_msgs::msg::DiagnosticArray array;
    array.header.stamp = now();

    diagnostic_msgs::msg::DiagnosticStatus status;
    status.name = "parking_input_evaluator";
    status.hardware_id = "autoware";

    switch (report.worst_level) {
      case Level::FAIL:
        status.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
        break;
      case Level::WARN:
        status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
        break;
      default:
        status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
        break;
    }

    status.message =
      std::string("input_ready=") + (report.is_input_ready ? "true" : "false") + " goal_ok=" +
      (report.can_plan_current_goal ? "true" : "false") + " pass=" +
      std::to_string(report.pass_count) + " warn=" + std::to_string(report.warn_count) +
      " fail=" + std::to_string(report.fail_count);

    for (const auto & item : report.checks) {
      if (item.level == Level::PASS) {
        continue;
      }
      diagnostic_msgs::msg::KeyValue value;
      value.key = std::string(toString(item.level)) + "_" + item.id;
      value.value = item.title + ": " + item.detail;
      status.values.push_back(std::move(value));
    }

    array.status.push_back(std::move(status));
    diag_pub_->publish(array);
  }

  void publishMarkers(const Report & report)
  {
    const geometry_msgs::msg::Pose goal = has_route_ ? route_goal_ : geometry_msgs::msg::Pose{};
    const auto markers = buildMarkers(
      report, grid_, map_, goal, has_route_,
      grid_.info.frame_id.empty() ? "map" : grid_.info.frame_id, now());
    marker_pub_->publish(markers);
  }

  // ---------------------------------------------------------------- members
  Params params_;

  GridData grid_;
  bool has_grid_{false};
  bool grid_origin_identity_{false};
  uint32_t grid_count_{0};
  std::deque<double> grid_stamps_;

  bool has_route_{false};
  geometry_msgs::msg::Pose route_goal_;
  geometry_msgs::msg::Pose route_start_;
  bool has_odom_at_route_{false};
  geometry_msgs::msg::Pose odom_at_route_;

  bool has_odom_{false};
  nav_msgs::msg::Odometry odom_;
  std::deque<double> odom_stamps_;

  std::deque<std::string> scenario_history_;

  MapGeometry map_;
  bool map_loaded_{false};

  Report last_report_;
  Level last_published_level_{Level::UNKNOWN};
  int stable_count_{0};

  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr grid_sub_;
  rclcpp::Subscription<autoware_planning_msgs::msg::LaneletRoute>::SharedPtr route_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<autoware_internal_planning_msgs::msg::Scenario>::SharedPtr scenario_sub_;

  rclcpp::Publisher<msg::ParkingInputReport>::SharedPtr report_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diag_pub_;
  rclcpp::Service<srv::GetParkingInputReport>::SharedPtr service_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

}  // namespace parking_input_evaluator

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  auto node = std::make_shared<parking_input_evaluator::ParkingInputEvaluatorNode>(options);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
