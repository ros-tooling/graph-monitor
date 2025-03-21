# ROS 2 Health Monitoring Components

A collection of packages for monitoring the health of an entire ROS 2 application.

As presented at ROSCon 2024 in "ROS robot health monitoring, the Bonsai approach": https://vimeo.com/1024971769

Also see https://github.com/BonsaiRobotics/roscon2024_health_sample for a sample application setting up these components.

See each package's README for more detailed information.

Packages here:
* [rmw_stats_shim](./rmw_stats_shim/) - RMW wrapper to gather and report global topic statistics
* [rosgraph_monitor](./rosgraph_monitor/) - Single component to monitor the ROS graph as a whole and publish diagnostics
* [rosgraph_monitor_msgs](./rosgraph_monitor_msgs/) - Messages for reporting graph monitoring information
* [telegraf_bridge](./telegraf_bridge/) - Node to collect local ROS information and forward to Telegraf for using in backend timeseries infrastructure

Also note:
* [patched rmw_implementation](https://github.com/BonsaiRobotics/rmw_implementation)
  * Needed to use `rmw_stats_shim`
