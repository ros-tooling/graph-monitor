# ROS 2 Graph Monitor

The packages in this repository implement application-level health monitoring for a ROS 2 connected graph of nodes.

The components were originally presented at ROSCon 2024 in "ROS robot health monitoring, the Bonsai approach": https://vimeo.com/1024971769

See each package's README for more detailed information:
* [rosgraph_monitor](./rosgraph_monitor/) - Component to monitor the ROS graph and publish resulting diagnostics
* [rosgraph_monitor_msgs](./rosgraph_monitor_msgs/) - Messages for reporting graph monitoring information, namely topic statistics
* [rmw_stats_shim](./rmw_stats_shim/) - RMW wrapper to efficiently gather and report topic statistics for all nodes
