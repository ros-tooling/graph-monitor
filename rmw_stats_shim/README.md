# RMW Stats Shim

By using https://github.com/BonsaiRobotics/rmw_implementation, provides an implementation for `RMW_IMPLEMENTATION_WRAPPER`

Does the following:
* Wraps & intercepts some RMW API calls
* Calculates "published period" statistics for every Publisher
* Calculates "received period" statistics for every Subscription
* Calculates "take age" statistics for every Subscription
  * This is the different between publish system timestamp and the time at which `rmw_take` is called, so is the sum of network latency plus executor latency
* Creates a `/topic_statistics` Publisher for every Node and publishes to it periodically about the statistics from within the node


## Key points

* Requires no subscriptions, no extra copies, to do statistics on all topics
* Requires no modification to application code to use


## Configuration

Currently uses environment variables rather than Parameters system due to ease of integration.

* `ROS_TOPIC_STATISTICS_WINDOW_SIZE` - how many messages to use in rolling buffer for stats (stores only timestamp information, not contents data)
  * Default: `50`
* `ROS_TOPIC_STATISTICS_TOPIC_NAME` - name of the topic to publish statistics on
  * Default: `/topic_statistics`
* `ROS_TOPIC_STATISTICS_PUBLISH_PERIOD` - Interval in seconds at which to periodically publish stats
  * Default: `1.0`

## Usage

```
colcon build --packages-select rmw_implementation rmw_stats_shim
export RMW_IMPLEMENTATION=<whatever e.g. rmw_cyclonedds_cpp>
export RMW_IMPLEMENTATION_WRAPPER=rmw_stats_shim
# run any ROS 2 node/launch/etc
```
