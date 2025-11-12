# ROS 2 Graph Monitor

The packages in this repository implement monitoring for an entire ROS node graph.

See each package's README for information on its design and usage:

* [rosgraph_monitor](./rosgraph_monitor/) - A library and node to monitor the ROS graph, publish periodic snapshots, and emit diagnostics reflecting analyses of its health
* [rosgraph_monitor_msgs](./rosgraph_monitor_msgs/) - Messages for describing the ROS graph and topic statistics
* [rmw_stats_shim](./rmw_stats_shim/) - RMW wrapper to efficiently gather and report global topic statistics

## Release Status

<table width="100%">
  <tr>
    <th>ROS2 Distro</th>
    <th>Humble</th>
    <th>Jazzy</th>
    <th>Kilted</th>
    <th>Rolling</th>
  </tr>
  <tr>
    <th>Release status</th>
    <td> <!-- humble -->
      <a href='https://build.ros2.org/job/Hdev__graph_monitor__ubuntu_jammy_amd64/'><img src='https://build.ros2.org/job/Hdev__graph_monitor__ubuntu_jammy_amd64/badge/icon?subject=Dev'></a><br/>
      <a href='https://build.ros2.org/job/Hdoc__graph_monitor__ubuntu_jammy_amd64/'><img src='https://build.ros2.org/job/Hdoc__graph_monitor__ubuntu_jammy_amd64/badge/icon?subject=Doc'></a><br/>
      <a href='https://build.ros2.org/job/Hbin_uJ64__rmw_stats_shim__ubuntu_jammy_amd64__binary/'><img src='https://build.ros2.org/job/Hbin_uJ64__rmw_stats_shim__ubuntu_jammy_amd64__binary/badge/icon?subject=rmw_stats_shim'></a><br/>
      <a href='https://build.ros2.org/job/Hbin_uJ64__rosgraph_monitor__ubuntu_jammy_amd64__binary/'><img src='https://build.ros2.org/job/Hbin_uJ64__rosgraph_monitor__ubuntu_jammy_amd64__binary/badge/icon?subject=rosgraph_monitor'></a><br/>
      <a href='https://build.ros2.org/job/Hbin_uJ64__rosgraph_monitor_msgs__ubuntu_jammy_amd64__binary/'><img src='https://build.ros2.org/job/Hbin_uJ64__rosgraph_monitor_msgs__ubuntu_jammy_amd64__binary/badge/icon?subject=rosgraph_monitor_msgs'></a>
    </td>
    <td> <!-- jazzy -->
      <a href='https://build.ros2.org/job/Jdev__graph_monitor__ubuntu_noble_amd64/'><img src='https://build.ros2.org/job/Jdev__graph_monitor__ubuntu_noble_amd64/badge/icon?subject=Dev'></a><br/>
      <a href='https://build.ros2.org/job/Jdoc__graph_monitor__ubuntu_noble_amd64/'><img src='https://build.ros2.org/job/Jdoc__graph_monitor__ubuntu_noble_amd64/badge/icon?subject=Doc'></a><br/>
      <a href='https://build.ros2.org/job/Jbin_uN64__rmw_stats_shim__ubuntu_noble_amd64__binary/'><img src='https://build.ros2.org/job/Jbin_uN64__rmw_stats_shim__ubuntu_noble_amd64__binary/badge/icon?subject=rmw_stats_shim'></a><br/>
      <a href='https://build.ros2.org/job/Jbin_uN64__rosgraph_monitor__ubuntu_noble_amd64__binary/'><img src='https://build.ros2.org/job/Jbin_uN64__rosgraph_monitor__ubuntu_noble_amd64__binary/badge/icon?subject=rosgraph_monitor'></a><br/>
      <a href='https://build.ros2.org/job/Jbin_uN64__rosgraph_monitor_msgs__ubuntu_noble_amd64__binary/'><img src='https://build.ros2.org/job/Jbin_uN64__rosgraph_monitor_msgs__ubuntu_noble_amd64__binary/badge/icon?subject=rosgraph_monitor_msgs'></a>
    </td>
    <td> <!-- kilted -->
      <a href='https://build.ros2.org/job/Kdev__graph_monitor__ubuntu_noble_amd64/'><img src='https://build.ros2.org/job/Kdev__graph_monitor__ubuntu_noble_amd64/badge/icon?subject=Dev'></a><br/>
      <a href='https://build.ros2.org/job/Kdoc__graph_monitor__ubuntu_noble_amd64/'><img src='https://build.ros2.org/job/Kdoc__graph_monitor__ubuntu_noble_amd64/badge/icon?subject=Doc'></a><br/>
      <a href='https://build.ros2.org/job/Kbin_uN64__rmw_stats_shim__ubuntu_noble_amd64__binary/'><img src='https://build.ros2.org/job/Kbin_uN64__rmw_stats_shim__ubuntu_noble_amd64__binary/badge/icon?subject=rmw_stats_shim'></a><br/>
      <a href='https://build.ros2.org/job/Kbin_uN64__rosgraph_monitor__ubuntu_noble_amd64__binary/'><img src='https://build.ros2.org/job/Kbin_uN64__rosgraph_monitor__ubuntu_noble_amd64__binary/badge/icon?subject=rosgraph_monitor'></a><br/>
      <a href='https://build.ros2.org/job/Kbin_uN64__rosgraph_monitor_msgs__ubuntu_noble_amd64__binary/'><img src='https://build.ros2.org/job/Kbin_uN64__rosgraph_monitor_msgs__ubuntu_noble_amd64__binary/badge/icon?subject=rosgraph_monitor_msgs'></a>
    </td>
    <td> <!-- rolling -->
      <a href='https://build.ros2.org/job/Rdev__graph_monitor__ubuntu_noble_amd64/'><img src='https://build.ros2.org/job/Rdev__graph_monitor__ubuntu_noble_amd64/badge/icon?subject=Dev'></a><br/>
      <a href='https://build.ros2.org/job/Rdoc__graph_monitor__ubuntu_noble_amd64/'><img src='https://build.ros2.org/job/Rdoc__graph_monitor__ubuntu_noble_amd64/badge/icon?subject=Doc'></a><br/>
      <a href='https://build.ros2.org/job/Rbin_uN64__rmw_stats_shim__ubuntu_noble_amd64__binary/'><img src='https://build.ros2.org/job/Rbin_uN64__rmw_stats_shim__ubuntu_noble_amd64__binary/badge/icon?subject=rmw_stats_shim'></a><br/>
      <a href='https://build.ros2.org/job/Rbin_uN64__rosgraph_monitor__ubuntu_noble_amd64__binary/'><img src='https://build.ros2.org/job/Rbin_uN64__rosgraph_monitor__ubuntu_noble_amd64__binary/badge/icon?subject=rosgraph_monitor'></a><br/>
      <a href='https://build.ros2.org/job/Rbin_uN64__rosgraph_monitor_msgs__ubuntu_noble_amd64__binary/'><img src='https://build.ros2.org/job/Rbin_uN64__rosgraph_monitor_msgs__ubuntu_noble_amd64__binary/badge/icon?subject=rosgraph_monitor_msgs'></a>
    </td>
  </tr>
</table>

## Visualization

This repository's scope is monitoring, reporting, and analysis of the ROS graph.

A Foxglove extension is [available in the public registry](https://github.com/polymathrobotics/foxglove_extensions/tree/main/ros2-graph) that can visualize the the `rosgraph_monitor_msgs/Graph` message type that is published on the topic `/rosgraph` by the monitor node.

## Similar projects

Here are some projects we have found that are trying achieve the same or similar goals.
We keep this list with the hopes of collaborating with those maintainers on identifying feature gaps here that can be added.

* https://github.com/nilseuropa/ros2graph_explorer
* https://github.com/Eight-Vectors/ros2-studio-vscode-plugin

Both of these projects provide more advanced visualizations than the Foxglove plugin, at the time of this writing, but less flexible architectures.
We are especially interested in seeing how we can pull out one generically visualization component from all this.

## History

These code components were originally presented at ROSCon 2024 in "ROS robot health monitoring, a Bonsai approach": https://vimeo.com/1024971769

They have since been expanded and are under ongoing development in the scope of the ROSGraph Working Group.
