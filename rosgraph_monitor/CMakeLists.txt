# Copyright 2024, Bonsai Robotics, Inc - All Rights Reserved
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

cmake_minimum_required(VERSION 3.8)
project(rosgraph_monitor)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Werror=switch)
  add_link_options("-Wl,--no-undefined")
endif()

find_package(ament_cmake REQUIRED)
find_package(generate_parameter_library REQUIRED)

find_package(diagnostic_msgs REQUIRED)
find_package(diagnostic_updater REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rosgraph_monitor_msgs REQUIRED)

message(STATUS "Detected ROS Distro $ENV{ROS_DISTRO}")
if("$ENV{ROS_DISTRO}" STREQUAL "humble")
  message(STATUS "Setting compile definition for ROS 2 Humble")
  add_compile_definitions("ROS2_HUMBLE")
endif()


# Library
add_library(${PROJECT_NAME} SHARED
  src/event.cpp
  src/monitor.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")
target_link_libraries(${PROJECT_NAME} PUBLIC
  diagnostic_updater::diagnostic_updater
  rclcpp::rclcpp
  ${diagnostic_msgs_TARGETS}
  ${rosgraph_monitor_msgs_TARGETS}
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
  rclcpp_components::component
  ${diagnostic_msgs_TARGETS}
  ${rosgraph_monitor_msgs_TARGETS}
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
  find_package(ament_cmake_gmock REQUIRED)
  find_package(ament_lint_auto REQUIRED)

  ament_lint_auto_find_test_dependencies()

  ament_add_gmock(test_graph_monitor test/test_graph_monitor.cpp)
  target_link_libraries(test_graph_monitor ${PROJECT_NAME})
endif()

ament_export_targets(${PROJECT_NAME}Targets HAS_LIBRARY_TARGET)
ament_package()
