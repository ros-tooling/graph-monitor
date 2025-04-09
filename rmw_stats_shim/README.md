# RMW Stats Shim

Depends on a [modified version of `rmw_implementation`](https://github.com/robograph-project/rmw_implementation) to provide topic statistics calculation within the RMW layer of each node, without having to instrument application code.

This component
* Wraps & intercepts some RMW API calls
* Calculates "published period" statistics for every Publisher
* Calculates "received period" statistics for every Subscription
* Calculates "take age" statistics for every Subscription
  * This is the different between publish system timestamp and the time at which `rmw_take` is called, so is the sum of network latency plus executor latency
* Creates a `/topic_statistics` Publisher for every Node and publishes to it periodically about the statistics from within the node

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
