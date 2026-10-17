# Rosgraph Monitor Parameters

Default Config

```yaml
rosgraph_monitor:
  ros__parameters:
    diagnostics_publish_period_ms: 1000.0
    graph_monitor:
      continuity:
        enable: true
        ignore_subscriber_nodes: '{/rosbag2_recorder, /foxglove_bridge}'
        ignore_topic_names: '{/rosout}'
        ignore_topic_types: '{foxglove_msgs/msg/ImageAnnotations, foxglove_msgs/msg/SceneUpdate,
          visualization_msgs/msg/Marker, visualization_msgs/msg/MarkerArray}'
      diagnostic_namespace: rosgraph
      nodes:
        ignore_prefixes: '{/_ros2cli_, /dummy, /launch_ros_, /transform_listener_impl_,
          /rqt_, /rviz_}'
        warn_only_prefixes: '{}'
      topic_statistics:
        deadline_allowed_error: 0.1
        ignore_topics: '{}'
        mandatory_topics: '{}'
        stale_timeout_ms: 3000.0

```

## diagnostics_publish_period_ms

Period in milliseconds between each publishing of the graph diagnostics.

* Type: `int`

* Default Value: 1000
* Read only: True

## graph_monitor.diagnostic_namespace

Namespace to put up front of generated diagnostics

* Type: `string`

* Default Value: "rosgraph"

## graph_monitor.nodes.ignore_prefixes

If node FQN starts with any of these prefixes, ignore completely for graph analysis

* Type: `string_array`

* Default Value: {"/_ros2cli_", "/dummy", "/launch_ros_", "/transform_listener_impl_", "/rqt_", "/rviz_"}

## graph_monitor.nodes.warn_only_prefixes

If node FQN starts with any of these prefixes, downgrade ERROR to WARN when missing

* Type: `string_array`

* Default Value: {}

## graph_monitor.continuity.enable

Set to false to disable continuity checking (maybe you have a ton of debug topics?)

* Type: `bool`

* Default Value: true

## graph_monitor.continuity.ignore_subscriber_nodes

For continuity checking, pretend these nodes are not subscribing to any topics

* Type: `string_array`

* Default Value: {"/rosbag2_recorder", "/foxglove_bridge"}

## graph_monitor.continuity.ignore_topic_types

Never report discontinuities on topics with these types - useful for viz/debug messages

* Type: `string_array`

* Default Value: {"foxglove_msgs/msg/ImageAnnotations", "foxglove_msgs/msg/SceneUpdate", "visualization_msgs/msg/Marker", "visualization_msgs/msg/MarkerArray"}

## graph_monitor.continuity.ignore_topic_names

Never report discontinuities on topics with these exact names

* Type: `string_array`

* Default Value: {"/rosout"}

## graph_monitor.topic_statistics.deadline_allowed_error

Multiplier for the deadline period, if period falls within this error bound it is considered correct

* Type: `double`

* Default Value: 0.1

## graph_monitor.topic_statistics.stale_timeout_ms

Maximum period to receive updated statistics for a topic before an error is raised that it is missing

* Type: `int`

* Default Value: 3000

## graph_monitor.topic_statistics.mandatory_topics

Pre-seeds list of topics that must exist, have deadlines, and meet them.
Note all other discovered deadline topics are still tracked.

* Type: `string_array`

* Default Value: {}

## graph_monitor.topic_statistics.ignore_topics

Topics with a deadline whose frequency should not be considered for health outcomes.
This is mainly an escape hatch for known bad design/implementation

* Type: `string_array`

* Default Value: {}
