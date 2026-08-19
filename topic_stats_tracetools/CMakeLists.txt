# SPDX-FileCopyrightText: 2026 Polymath Robotics, Inc.
# SPDX-License-Identifier: Apache-2.0
cmake_minimum_required(VERSION 3.22)
project(topic_stats_tracetools)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Werror=switch)
  add_link_options("-Wl,--no-undefined")
endif()

find_package(ament_cmake REQUIRED)
find_package(topic_stats_core REQUIRED)
find_package(topic_stats_shm REQUIRED)

# The adapter logic, free of any interposition. This is what the tests drive.
add_library(${PROJECT_NAME} SHARED
  src/trace_collector.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")
target_link_libraries(${PROJECT_NAME} PUBLIC
  topic_stats_core::topic_stats_core
  topic_stats_shm::topic_stats_shm
)

# The preloadable library. Separate from the one above so that tests link the logic without
# defining ros_trace_* symbols in the test binary, which would interpose on the test itself.
add_library(${PROJECT_NAME}_preload SHARED
  src/interpose.cpp
)
target_link_libraries(${PROJECT_NAME}_preload PUBLIC
  ${PROJECT_NAME}
  ${CMAKE_DL_LIBS}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gmock REQUIRED)

  ament_add_gmock(test_trace_collector test/test_trace_collector.cpp)
  target_link_libraries(test_trace_collector ${PROJECT_NAME})
endif()

ament_export_targets(${PROJECT_NAME}Targets HAS_LIBRARY_TARGET)
ament_export_dependencies(topic_stats_core topic_stats_shm)

install(
  DIRECTORY include/
  DESTINATION include
)
install(
  TARGETS
    ${PROJECT_NAME}
    ${PROJECT_NAME}_preload
  EXPORT ${PROJECT_NAME}Targets
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

ament_package()
