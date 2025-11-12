# Rosgraph Monitor

Provides a C++ library `rosgraph_monitor` with main class `rosgraph_monitor::RosgraphMonitor`, which watches the entire ROS application to create an up-to-date graph representation, running analyses to check whether it meets certain "healthy" criteria.

The library may be used directly by application code, or run standalone as a parameter-configurable ROS Node `rosgraph_monitor_node`.

Either way it's used, the Graph Monitor provides a set of [`diagnostic_msgs/DiagnosticStatus`](https://docs.ros.org/en/rolling/p/diagnostic_msgs/msg/DiagnosticStatus.html) summarizing the graph health.
These Diagnostics are provided in a friendly format for [`DiagnosticAggregator`](https://docs.ros.org/en/rolling/p/diagnostic_aggregator/) to interpret as a toplevel health status.

A [convenience launch file](./launch/monitor_launch.yaml) is provided which starts the graph monitor node and a properly configured diagnostic aggregator, and can be used directly or as an example for usage pattern.

## Graph Representation

The monitor keeps track of the following entities and their relationships:

- Node
  - name
  - parameters
  - publishers
  - subscriptions
- Parameter
  - name
  - type
- Publisher
  - node
  - name
  - type
  - QoS profile
- Subscription
  - node
  - name
  - type
  - QoS profile

## Health Checks

These health criteria are provided:

- Node Liveness: Report errors for nodes that were once present but are no longer in the graph
- Graph Continuity: report warnings for publishers or subscriptions that don't have anything listening/sending on the other end
- [Topic Frequencies](#frequency-checks): interpret `/topic_statistics` to determine whether certain topics aren't going at the expected rate. See following section for more information.

This also provides a `rosgraph_monitor_node` for standalone use, for users who do not need custom integration of the `RosgraphMonitor` class.

### Frequency Checks

To set expectations for Topic Statistics Diagnostics, you must:

1. Enable `/topic_statistics`. See [rmw_stats_shim](../rmw_stats_shim/) for how to do this
1. Publishers that are expected to send at a fixed rate must set the Deadline QoS policy
1. Subscriptions do not need any special treatment

Note that the Deadline describes the expected _period_ of time between messages.
This is the inverse of _frequency_.
For example, if you expect your publisher to be publishing at 10Hz, then its Deadline is 0.1 seconds.

The monitor is only using Deadline as a reporting mechanism for expected frequency of publishing - the underlying RMW implementation of the policy is not relevant.
You do _not_ need to set the deadline on Subscriptions, or register any callbacks such as `DeadlineMissed`

## Usage

### API

See [`RosGraphMonitor`](./include/rosgraph_monitor/monitor.hpp#L99)

### Node

Recommended: run the provided launchfile, which also launches a `DiagnosticsAggregator` to provide an appropriately interpreted `/diagnostics_agg` and `/diagnostics_toplevel_state`.

```bash
ros2 launch rosgraph_monitor monitor_launch.yaml
```

But you may also just run the standalone node, if you want to perform separate diagnostics aggregation

```bash
ros2 run rosgraph_monitor rosgraph_monitor_node
```

### Configuration

See [params_decl.yaml](./src/params_decl.yaml) for the full parameters for the `rosgraph_monitor` Node.

These parameters mirror the structure [`rosgraph_monitor::GraphMonitorConfiguration`](./include/rosgraph_monitor/monitor.hpp#L60), for those using the class directly instead of in the wrapping Node.

## Visualization

A Foxglove extension is provided that can visualize the the `rosgraph_monitor_msgs/Graph` message type that is published on the topic `/rosgraph` by the monitor node.

https://github.com/polymathrobotics/foxglove_extensions/tree/main/ros2-graph

## Implementation Notes

The `RosgraphMonitor` class is built to not initiate any ROS communications, which means it can be used in any preexisting program or node.
It is passed a `NodeGraphInterface` abstract class pointer to make queries about the graph structure, and is passed `TopicStatistics` messages.
