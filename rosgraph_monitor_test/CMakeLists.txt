# SPDX-FileCopyrightText: 2025 Polymath Robotics, Inc.
# SPDX-License-Identifier: Apache-2.0
cmake_minimum_required(VERSION 3.8)
project(rosgraph_monitor_test)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

# Matches rosgraph_monitor, so tests can guard on distro-specific API
if(NOT DEFINED ENV{ROS_DISTRO})
  message(FATAL_ERROR "ROS_DISTRO environment variable not set")
else()
  string(TOUPPER "$ENV{ROS_DISTRO}" ROS_DISTRO)
  message(STATUS "Detected ROS 2 distro ${ROS_DISTRO} from environment")
  add_compile_definitions("ROS2_${ROS_DISTRO}")
endif()

ament_python_install_package(${PROJECT_NAME})

if(BUILD_TESTING)
  ament_auto_find_test_dependencies()

  add_launch_test(test/test_graph_monitor_launch.py)
  add_launch_test(test/test_diagnostics_launch.py)

  ament_add_gtest(test_qos_profile_constants
    test/test_qos_profile_constants.cpp
  )
  target_link_libraries(test_qos_profile_constants
    rclcpp::rclcpp
    rosgraph_msgs::rosgraph_msgs
  )
endif()

ament_package()
