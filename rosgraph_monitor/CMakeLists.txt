# SPDX-FileCopyrightText: 2024 Bonsai Robotics, Inc.
# SPDX-License-Identifier: Apache-2.0
cmake_minimum_required(VERSION 3.22)
project(rosgraph_monitor)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Werror=switch)
  add_link_options("-Wl,--no-undefined")
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

if(NOT DEFINED ENV{ROS_DISTRO})
  message(FATAL_ERROR "ROS_DISTRO environment variable not set")
else()
  string(TOUPPER "$ENV{ROS_DISTRO}" ROS_DISTRO)
  message(STATUS "Detected ROS 2 distro ${ROS_DISTRO} from environment")
  add_compile_definitions("ROS2_${ROS_DISTRO}")
endif()

# Library
add_library(${PROJECT_NAME} SHARED
  src/event.cpp
  src/monitor.cpp
  src/rclcpp_parameter_client.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")
target_link_libraries(${PROJECT_NAME} PUBLIC
  diagnostic_msgs::diagnostic_msgs
  diagnostic_updater::diagnostic_updater
  rclcpp::rclcpp
  rosgraph_monitor_msgs::rosgraph_monitor_msgs
  rosgraph_msgs::rosgraph_msgs
)

generate_parameter_library(${PROJECT_NAME}_generated_parameters
  src/params_decl.yaml
)

# Node
add_library(${PROJECT_NAME}_nodelib SHARED
  src/node.cpp
)
target_include_directories(${PROJECT_NAME}_nodelib PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")
target_link_libraries(${PROJECT_NAME}_nodelib PUBLIC
  ${PROJECT_NAME}
  ${PROJECT_NAME}_generated_parameters
  diagnostic_msgs::diagnostic_msgs
  rclcpp_components::component
  rosgraph_monitor_msgs::rosgraph_monitor_msgs
  rosgraph_msgs::rosgraph_msgs
)

rclcpp_components_register_node(${PROJECT_NAME}_nodelib
  PLUGIN "rosgraph_monitor::Node"
  EXECUTABLE ${PROJECT_NAME}_node
)

install(
  DIRECTORY include/
  DESTINATION include
)
install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)
install(
  TARGETS
    ${PROJECT_NAME}
    ${PROJECT_NAME}_generated_parameters
    ${PROJECT_NAME}_nodelib
  EXPORT ${PROJECT_NAME}Targets
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

if(BUILD_TESTING)
  ament_auto_find_test_dependencies()

  ament_add_gmock(test_graph_monitor test/test_graph_monitor.cpp)
  target_link_libraries(test_graph_monitor ${PROJECT_NAME})

  ament_add_gmock(test_event test/test_event.cpp)
  target_link_libraries(test_event ${PROJECT_NAME})

  ament_add_gmock(test_mutex_protected test/test_mutex_protected.cpp)
  target_link_libraries(test_mutex_protected ${PROJECT_NAME})

  ament_add_gmock(test_query_queue test/test_query_queue.cpp)
  target_link_libraries(test_query_queue ${PROJECT_NAME})
endif()

ament_export_targets(${PROJECT_NAME}Targets HAS_LIBRARY_TARGET)
ament_package()
