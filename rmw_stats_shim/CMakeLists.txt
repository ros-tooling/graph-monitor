# SPDX-FileCopyrightText: 2024 Bonsai Robotics, Inc.
# SPDX-License-Identifier: Apache-2.0
cmake_minimum_required(VERSION 3.22)
project(rmw_stats_shim)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
  add_link_options("-Wl,--no-undefined")
endif()

find_package(ament_cmake REQUIRED)

find_package(rcpputils REQUIRED)
find_package(rmw REQUIRED)
find_package(rosgraph_monitor_msgs REQUIRED)
find_package(rosidl_runtime_cpp REQUIRED)
find_package(rosidl_typesupport_cpp REQUIRED)
find_package(topic_stats_core REQUIRED)
find_package(Threads REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/rmw_publisher_sink.cpp
  src/shim.cpp
  src/stat_collector.cpp
  src/stats_message.cpp
  src/timer.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")
target_link_libraries(${PROJECT_NAME} PUBLIC
  rcpputils::rcpputils
  rmw::rmw
  rosidl_runtime_cpp::rosidl_runtime_cpp
  rosidl_typesupport_cpp::rosidl_typesupport_cpp
  topic_stats_core::topic_stats_core
  Threads::Threads
  ${rosgraph_monitor_msgs_TARGETS}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gmock REQUIRED)

  # Both suites cover the parts of this adapter that do not need a middleware: the handle mapping
  # and the conversion to the outgoing message.
  ament_add_gmock(test_handle_map test/test_handle_map.cpp)
  target_link_libraries(test_handle_map ${PROJECT_NAME})

  ament_add_gmock(test_stats_message test/test_stats_message.cpp)
  target_link_libraries(test_stats_message ${PROJECT_NAME})
endif()

ament_export_targets(${PROJECT_NAME}Targets HAS_LIBRARY_TARGET)
install(
  DIRECTORY include/
  DESTINATION include
)
install(
  TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}Targets
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

ament_package()
