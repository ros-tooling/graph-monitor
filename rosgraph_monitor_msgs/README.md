# rosgraph_monitor_msgs

Provides messages for reporting statistics measured about topic endpoints:

* [TopicStatistic.msg](./msg/TopicStatistic.msg) - A single statistic about one endpoint (Publisher/Subscription)
* [TopicStatistics.msg](./msg/TopicStatistics.msg) - A timestamped array of `TopicStatistic`, for a Node to report periodically in bulk about all its endpoints

Messages describing the graph itself (`Graph`, `Node`, `Topic`, `QoSProfile`) live upstream in
[`rosgraph_msgs`](https://github.com/ros2/rcl_interfaces/tree/rolling/rosgraph_msgs).
