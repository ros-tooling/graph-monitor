# SPDX-FileCopyrightText: 2026 Polymath Robotics, Inc.
# SPDX-License-Identifier: Apache-2.0
cmake_minimum_required(VERSION 3.22)
project(topic_stats_shm)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Werror=switch)
  add_link_options("-Wl,--no-undefined")
endif()

find_package(ament_cmake REQUIRED)
find_package(topic_stats_core REQUIRED)
find_package(Threads REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/segment.cpp
  src/shm_reader.cpp
  src/shm_sink.cpp
  src/wire_format.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")
target_link_libraries(${PROJECT_NAME} PUBLIC
  topic_stats_core::topic_stats_core
  Threads::Threads
)
# shm_open lives in librt on older glibc.
target_link_libraries(${PROJECT_NAME} PRIVATE ${CMAKE_DL_LIBS} rt)

if(BUILD_TESTING)
  find_package(ament_cmake_gmock REQUIRED)

  ament_add_gmock(test_wire_format test/test_wire_format.cpp)
  target_link_libraries(test_wire_format ${PROJECT_NAME})

  ament_add_gmock(test_segment test/test_segment.cpp)
  target_link_libraries(test_segment ${PROJECT_NAME})

  ament_add_gmock(test_round_trip test/test_round_trip.cpp)
  target_link_libraries(test_round_trip ${PROJECT_NAME})
endif()

ament_export_targets(${PROJECT_NAME}Targets HAS_LIBRARY_TARGET)
ament_export_dependencies(topic_stats_core Threads)

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
