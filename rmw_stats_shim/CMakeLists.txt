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
project(rmw_stats_shim)

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

add_library(${PROJECT_NAME} SHARED
  src/shim.cpp
  src/stat_collector.cpp
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
  ${rosgraph_monitor_msgs_TARGETS}
)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
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
