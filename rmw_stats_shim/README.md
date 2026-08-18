# RMW Stats Shim

Depends on a [modified version of `rmw_implementation`](https://github.com/ros-tooling/rmw_implementation) to provide topic statistics calculation within the RMW layer of each node, without having to instrument application code.

This component
* Wraps & intercepts some RMW API calls
* Calculates "published period" statistics for every Publisher
* Calculates "received period" statistics for every Subscription
* Calculates "take age" statistics for every Subscription
  * This is the different between publish system timestamp and the time at which `rmw_take` is called, so is the sum of network latency plus executor latency
* Creates a `/topic_statistics` Publisher for every Node and publishes to it periodically about the statistics from within the node

## Structure

The statistics themselves live in `topic_stats_core`, which has no ROS dependencies. This package is
the RMW-specific half, split along the two boundaries that package defines:

* `stat_collector.*` is the ingest adapter. It translates intercepted RMW calls into
  `topic_stats_core::Recorder` calls and owns nothing but the mapping from RMW handles to core ids.
* `handle_map.hpp` is that mapping. RMW entities are created and destroyed on arbitrary threads
  while messages flow on others, so it is synchronized, and it is testable without a middleware.
* `rmw_publisher_sink.*` is the egress adapter, a `topic_stats_core::StatsSink` that gives each node
  a `/topic_statistics` publisher created directly against the RMW implementation. Creating and
  destroying those publishers emits graph events that re-enter this shim, so their lifetime is
  managed with no lock of this package held.
* `stats_message.*` converts a core snapshot into `rosgraph_monitor_msgs`. Kept separate from
  publishing so it can be tested on its own.

A second ingest adapter based on the `ros_trace_*` functions that `rcl` and the RMW implementations
already call is the reason for the split: it can share everything below `Recorder` without needing a
patched `rmw_implementation`.

Key points:
* Requires no subscriptions, no extra copies, to do statistics on all topics
* Requires no modification to application code to use

## Configuration

Set the following environment variables before launching a node to configure topic statistics

* Required: `RMW_IMPLEMENTATION_WRAPPER=rmw_stats_shim`
* `ROS_TOPIC_STATISTICS_WINDOW_SIZE` - how many messages to use in rolling buffer for stats (stores only timestamp information, not contents data)
  * Default: `50`
* `ROS_TOPIC_STATISTICS_TOPIC_NAME` - name of the topic to publish statistics on
  * Default: `/topic_statistics`
* `ROS_TOPIC_STATISTICS_PUBLISH_PERIOD` - Interval in seconds at which to periodically publish stats
  * Default: `1.0`

## Usage

```
colcon build --packages-select rmw_implementation rmw_stats_shim
source install/setup.bash
export RMW_IMPLEMENTATION_WRAPPER=rmw_stats_shim
# run any ROS 2 node/launch/etc
```
