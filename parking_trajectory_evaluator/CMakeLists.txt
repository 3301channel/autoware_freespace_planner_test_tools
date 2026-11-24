cmake_minimum_required(VERSION 3.14)
project(parking_trajectory_evaluator)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -O2)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

find_package(autoware_planning_msgs REQUIRED)
find_package(autoware_internal_planning_msgs REQUIRED)

# ---------------------------------------------------------------- interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/MetricValue.msg"
  "msg/ParkingTrajectoryReport.msg"
  "srv/GetParkingReport.srv"
  DEPENDENCIES std_msgs geometry_msgs
)

rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")

# ---------------------------------------------------------------- core lib
# Pure C++ metrics. No ROS types, so it is straightforward to unit test.
add_library(${PROJECT_NAME}_core SHARED
  src/trajectory_metrics.cpp
  src/collision_checker.cpp
  src/scoring.cpp
)
target_include_directories(${PROJECT_NAME}_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# ---------------------------------------------------------------- node
add_executable(${PROJECT_NAME}_node
  src/parking_trajectory_evaluator_node.cpp
  src/visualization.cpp
)
target_link_libraries(${PROJECT_NAME}_node
  ${PROJECT_NAME}_core
  "${cpp_typesupport_target}"
  rclcpp::rclcpp
  ${std_msgs_TARGETS}
  ${geometry_msgs_TARGETS}
  ${nav_msgs_TARGETS}
  ${diagnostic_msgs_TARGETS}
  ${visualization_msgs_TARGETS}
  autoware_planning_msgs::autoware_planning_msgs
  autoware_internal_planning_msgs::autoware_internal_planning_msgs
)

# ---------------------------------------------------------------- install
install(TARGETS ${PROJECT_NAME}_node
  EXPORT export_${PROJECT_NAME}
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(TARGETS ${PROJECT_NAME}_core
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME}_core)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_trajectory_metrics test/test_trajectory_metrics.cpp)
  target_link_libraries(test_trajectory_metrics ${PROJECT_NAME}_core)

  ament_add_gtest(test_scoring test/test_scoring.cpp)
  target_link_libraries(test_scoring ${PROJECT_NAME}_core)

  ament_add_gtest(test_collision_checker test/test_collision_checker.cpp)
  target_link_libraries(test_collision_checker ${PROJECT_NAME}_core)
endif()

ament_package()
