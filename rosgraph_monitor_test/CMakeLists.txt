cmake_minimum_required(VERSION 3.8)
project(rosgraph_monitor_test)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_python_install_package(${PROJECT_NAME})

if(BUILD_TESTING)
  ament_auto_find_test_dependencies()
  ament_lint_auto_find_test_dependencies()

  add_launch_test(test/test_graph_monitor_launch.py)
  add_launch_test(test/test_diagnostics_launch.py)

  ament_add_gtest(test_qos_profile_constants
    test/test_qos_profile_constants.cpp
  )
  target_link_libraries(test_qos_profile_constants
    rclcpp::rclcpp
    ${rosgraph_monitor_msgs_TARGETS}
  )
endif()

ament_package()
