# SPDX-FileCopyrightText: 2026 Polymath Robotics, Inc.
# SPDX-License-Identifier: Apache-2.0
cmake_minimum_required(VERSION 3.22)
project(topic_stats_collector)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Werror=switch)
  add_link_options("-Wl,--no-undefined")
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rosgraph_monitor_msgs REQUIRED)
find_package(topic_stats_core REQUIRED)
find_package(topic_stats_ros REQUIRED)
find_package(topic_stats_shm REQUIRED)

# The pool is separate from the node because process lifetime is the hard part here, and it is
# testable without a middleware.
add_library(${PROJECT_NAME} SHARED
  src/segment_pool.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")
target_link_libraries(${PROJECT_NAME} PUBLIC
  topic_stats_core::topic_stats_core
  topic_stats_shm::topic_stats_shm
)

add_library(${PROJECT_NAME}_nodelib SHARED
  src/node.cpp
)
target_include_directories(${PROJECT_NAME}_nodelib PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")
target_link_libraries(${PROJECT_NAME}_nodelib PUBLIC
  ${PROJECT_NAME}
  rclcpp::rclcpp
  rclcpp_components::component
  topic_stats_ros::topic_stats_ros
  ${rosgraph_monitor_msgs_TARGETS}
)

rclcpp_components_register_node(${PROJECT_NAME}_nodelib
  PLUGIN "topic_stats_collector::Node"
  EXECUTABLE ${PROJECT_NAME}_node
)

if(BUILD_TESTING)
  find_package(ament_cmake_gmock REQUIRED)

  ament_add_gmock(test_segment_pool test/test_segment_pool.cpp)
  target_link_libraries(test_segment_pool ${PROJECT_NAME})

  # Drives a real Collector down both egress paths and requires identical messages out.
  ament_add_gmock(test_egress_equivalence test/test_egress_equivalence.cpp)
  target_link_libraries(test_egress_equivalence ${PROJECT_NAME} topic_stats_ros::topic_stats_ros)
endif()

ament_export_targets(${PROJECT_NAME}Targets HAS_LIBRARY_TARGET)
ament_export_dependencies(topic_stats_core topic_stats_shm)

install(
  DIRECTORY include/
  DESTINATION include
)
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)
install(
  TARGETS
    ${PROJECT_NAME}
    ${PROJECT_NAME}_nodelib
  EXPORT ${PROJECT_NAME}Targets
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

ament_package()
