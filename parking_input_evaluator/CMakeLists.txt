cmake_minimum_required(VERSION 3.14)
project(parking_input_evaluator)

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
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

find_package(autoware_planning_msgs REQUIRED)
find_package(autoware_map_msgs REQUIRED)
find_package(autoware_internal_planning_msgs REQUIRED)

find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)

# tinyxml2 is used to read local_x / local_y tags, which the stock lanelet2
# OSM parser does not support.
find_path(TINYXML2_INCLUDE_DIR tinyxml2.h REQUIRED)
find_library(TINYXML2_LIBRARY tinyxml2 REQUIRED)

# ---------------------------------------------------------------- interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/CheckItem.msg"
  "msg/ParkingSpaceStatus.msg"
  "msg/ParkingInputReport.msg"
  "srv/GetParkingInputReport.srv"
  DEPENDENCIES std_msgs geometry_msgs
)

rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")

# ---------------------------------------------------------------- core lib
# Pure algorithm library: no ROS message dependency, easy to unit test.
add_library(${PROJECT_NAME}_core SHARED
  src/grid_utils.cpp
  src/connectivity.cpp
  src/map_parser.cpp
  src/pose_checker.cpp
)
target_include_directories(${PROJECT_NAME}_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(${PROJECT_NAME}_core PUBLIC
  ${OpenCV_LIBRARIES}
  Eigen3::Eigen
  ${TINYXML2_LIBRARY}
)
target_include_directories(${PROJECT_NAME}_core SYSTEM PUBLIC
  ${OpenCV_INCLUDE_DIRS}
  ${TINYXML2_INCLUDE_DIR}
)

# ---------------------------------------------------------------- node
add_executable(${PROJECT_NAME}_node
  src/parking_input_evaluator_node.cpp
  src/visualization.cpp
)
target_link_libraries(${PROJECT_NAME}_node
  ${PROJECT_NAME}_core
  "${cpp_typesupport_target}"
  rclcpp::rclcpp
  rclcpp_components::component
  ${std_msgs_TARGETS}
  ${geometry_msgs_TARGETS}
  ${nav_msgs_TARGETS}
  ${diagnostic_msgs_TARGETS}
  ${visualization_msgs_TARGETS}
  tf2::tf2
  tf2_ros::tf2_ros
)
target_link_libraries(${PROJECT_NAME}_node
  autoware_planning_msgs::autoware_planning_msgs
  autoware_map_msgs::autoware_map_msgs
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
  RUNTIME DESTINATION bin
)
install(DIRECTORY include/
  DESTINATION include
)
install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME}_core)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_grid_utils test/test_grid_utils.cpp)
  target_link_libraries(test_grid_utils ${PROJECT_NAME}_core)

  ament_add_gtest(test_connectivity test/test_connectivity.cpp)
  target_link_libraries(test_connectivity ${PROJECT_NAME}_core)

  ament_add_gtest(test_parking_space test/test_parking_space.cpp)
  target_link_libraries(test_parking_space ${PROJECT_NAME}_core)
endif()

ament_package()
