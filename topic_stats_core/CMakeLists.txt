# SPDX-FileCopyrightText: 2026 Polymath Robotics, Inc.
# SPDX-License-Identifier: Apache-2.0
cmake_minimum_required(VERSION 3.22)
project(topic_stats_core)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Werror=switch)
  add_link_options("-Wl,--no-undefined")
endif()

find_package(ament_cmake REQUIRED)
find_package(Threads REQUIRED)

# Intentionally depends on nothing from ROS. Ingest adapters bring rmw/tracetools, sinks bring
# whatever transport they speak, and this library stays testable without a middleware.
add_library(${PROJECT_NAME} SHARED
  src/collector.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")
target_link_libraries(${PROJECT_NAME} PUBLIC
  Threads::Threads
)

if(BUILD_TESTING)
  find_package(ament_cmake_gmock REQUIRED)

  ament_add_gmock(test_rolling_window test/test_rolling_window.cpp)
  target_link_libraries(test_rolling_window ${PROJECT_NAME})

  ament_add_gmock(test_slot_table test/test_slot_table.cpp)
  target_link_libraries(test_slot_table ${PROJECT_NAME})

  ament_add_gmock(test_collector test/test_collector.cpp)
  target_link_libraries(test_collector ${PROJECT_NAME})

  ament_add_gmock(test_collector_concurrency test/test_collector_concurrency.cpp)
  target_link_libraries(test_collector_concurrency ${PROJECT_NAME})
endif()

ament_export_targets(${PROJECT_NAME}Targets HAS_LIBRARY_TARGET)
ament_export_dependencies(Threads)

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
