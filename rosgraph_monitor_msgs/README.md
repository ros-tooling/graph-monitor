# rosgraph_monitor_msgs

Provides messages for communication about understanding gained from monitoring the ROS application graph:

* [TopicStatistic.msg](./msg/TopicStatistic.msg) - A single statistic about one endpoint (Publisher/Subscription)
* [TopicStatistics.msg](./msg/TopicStatistics.msg) - A timestamped array of `TopicStatistic`, for a Node to report periodically in bulk about all its endpoints
