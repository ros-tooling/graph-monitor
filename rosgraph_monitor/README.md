# Rosgraph Monitor

Provides a C++ class `RosgraphMonitor`, which uses a node's `NodeGraphInterface` and subscribes to `/topic_statistics` to monitor the ROS computation graph and produce `/diagnostics (DiagnosticArray)` about issues in the graph:
* Missing Nodes (nodes that were present and are now missing)
* Discontinuities in graph (publishers or subscriptions that have no counterparts)
* Data flow issues (topics not publishing at promised rate)
  * `/topic_statistics` required for this. See `rmw_stats_shim` for how those are produced

This also provides a `rosgraph_monitor_node` for standalone use, for users who do not need custom integration of the `RosgraphMonitor` class.

## Topic Statistics Diagnostics

To set expectations for Topic Statistics Diagnostics, you must:

1. Set the `Deadline` QoS policy on Publishers
1. Nothing else. You don't need to set it on Subscriptions, or register `DeadlineMissed` callbacks. Just state your periodic promise via Deadline QoS.

## Configuration

See [params_decl.yaml](./src/params_decl.yaml) for the full parameters for the `rosgraph_monitor` Node.

These parameters mirror the structure `rosgraph_monitor::GraphMonitorConfiguration` in [monitor.hpp](./include/rosgraph_monitor/monitor.hpp), for those using the class directly instead of in the wrapping Node.

## Usage

```
ros2 run rosgraph_monitor rosgraph_monitor_node
```
