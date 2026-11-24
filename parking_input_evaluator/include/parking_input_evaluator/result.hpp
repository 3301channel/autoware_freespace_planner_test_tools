// Copyright 2026
// Licensed under the Apache License, Version 2.0
#pragma once

#include <cstdint>
#include <limits>
#include <string>
#include <vector>

namespace parking_input_evaluator
{

enum class Level : uint8_t { UNKNOWN = 0, PASS = 1, WARN = 2, FAIL = 3 };

/// pass_ok : satisfies the strict (PASS) threshold
/// warn_ok : satisfies the relaxed (WARN) threshold
/// Both false -> FAIL.
inline Level classify(bool pass_ok, bool warn_ok)
{
  if (pass_ok) {
    return Level::PASS;
  }
  return warn_ok ? Level::WARN : Level::FAIL;
}

inline const char * toString(Level level)
{
  switch (level) {
    case Level::PASS:
      return "PASS";
    case Level::WARN:
      return "WARN";
    case Level::FAIL:
      return "FAIL";
    default:
      return "UNKNOWN";
  }
}

inline const char * toString(uint8_t level)
{
  return toString(static_cast<Level>(level));
}

inline double quietNaN()
{
  return std::numeric_limits<double>::quiet_NaN();
}

struct CheckItem
{
  std::string id;
  std::string title;
  Level level{Level::UNKNOWN};
  double value{0.0};
  double threshold_pass{quietNaN()};
  double threshold_warn{quietNaN()};
  std::string unit;
  std::string detail;
  std::string advice;
};

enum class SpaceStatus : uint8_t {
  PARKABLE = 0,
  OUT_OF_GRID = 1,
  BLOCKED = 2,
  UNREACHABLE = 3,
};

inline const char * toString(SpaceStatus status)
{
  switch (status) {
    case SpaceStatus::PARKABLE:
      return "PARKABLE";
    case SpaceStatus::OUT_OF_GRID:
      return "OUT_OF_GRID";
    case SpaceStatus::BLOCKED:
      return "BLOCKED";
    default:
      return "UNREACHABLE";
  }
}

struct SpaceResult
{
  uint64_t id{0};
  double cx{0.0};
  double cy{0.0};
  double heading{0.0};
  SpaceStatus status{SpaceStatus::PARKABLE};
  uint32_t conflict_samples{0};
  double available_width{0.0};
};

struct Report
{
  std::vector<CheckItem> checks;

  bool is_input_ready{false};
  bool can_plan_current_goal{false};
  Level worst_level{Level::UNKNOWN};

  uint32_t pass_count{0};
  uint32_t warn_count{0};
  uint32_t fail_count{0};
  std::vector<std::string> failure_reasons;

  double grid_rate_hz{0.0};
  double coverage_ratio{0.0};
  double unknown_ratio{0.0};
  double grid_extent[4]{0.0, 0.0, 0.0, 0.0};
  double lot_extent[4]{0.0, 0.0, 0.0, 0.0};
  double goal_edt_margin{0.0};
  double goal_rect_conflicts{0.0};

  uint32_t total_spaces{0};
  uint32_t parkable_spaces{0};
  uint32_t out_of_grid_spaces{0};
  uint32_t blocked_spaces{0};
  uint32_t unreachable_spaces{0};
  std::vector<SpaceResult> spaces;

  void addCheck(
    const std::string & id, const std::string & title, Level level, const std::string & detail,
    const std::string & advice = std::string())
  {
    CheckItem item;
    item.id = id;
    item.title = title;
    item.level = level;
    item.detail = detail;
    item.advice = advice;
    checks.push_back(std::move(item));
  }

  /// Finalize counters, booleans and the overall verdict.
  void finalize(bool current_goal_ok)
  {
    pass_count = warn_count = fail_count = 0;
    failure_reasons.clear();
    worst_level = Level::PASS;

    for (const auto & c : checks) {
      switch (c.level) {
        case Level::PASS:
          ++pass_count;
          break;
        case Level::WARN:
          ++warn_count;
          worst_level = Level::WARN;
          break;
        case Level::FAIL:
          ++fail_count;
          worst_level = Level::FAIL;
          failure_reasons.push_back(c.id + " " + c.title + ": " + c.detail);
          break;
        default:
          break;
      }
    }

    if (checks.empty()) {
      worst_level = Level::UNKNOWN;
    }

    // is_input_ready requires zero FAIL and zero WARN.
    is_input_ready = (fail_count == 0 && warn_count == 0);
    can_plan_current_goal = current_goal_ok && (fail_count == 0);
  }
};

}  // namespace parking_input_evaluator
