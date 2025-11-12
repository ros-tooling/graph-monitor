# rosgraph_monitor_msgs

Provides messages for communication about understanding gained from monitoring the ROS application graph:

* [Graph.msg](./msg/Graph.msg) - Represents an entire ROS application graph
* [Node.msg](./msg/Node.msg) - Information about a single Node
* [QosProfile.msg](./msg/QosProfile.msg) - Represents an RMW QoS Profile as a message. Values kept matched to underlying data type by [unit tests](../rosgraph_monitor_test/test/test_qos_profile_constants.cpp)
* [Topic.msg](./msg/Topic.msg) - Information about a topic endpoint, like a Publisher or Subscription
* [TopicStatistic.msg](./msg/TopicStatistic.msg) - A single statistic about one endpoint (Publisher/Subscription)
* [TopicStatistics.msg](./msg/TopicStatistics.msg) - A timestamped array of `TopicStatistic`, for a Node to report periodically in bulk about all its endpoints
