# rosgraph_monitor_test

Integration test package for the `rosgraph_monitor` ROS2 package.

## Overview

This package contains integration tests that verify the behavior of the `rosgraph_monitor` package in realistic ROS2 environments. It includes launch tests that spin up the actual monitor nodes and validate their ability to detect graph topology changes, node lifecycle events, and diagnostic status reporting.

## Usage

### Running Tests

```bash
# Run all launch tests
colcon test --packages-select rosgraph_monitor_test

# Run specific test file
launch_test src/graph-monitor/rosgraph_monitor_test/test/test_graph_monitor_launch.py

# Run with verbose output
launch_test --verbose src/graph-monitor/rosgraph_monitor_test/test/test_graph_monitor_launch.py
```
