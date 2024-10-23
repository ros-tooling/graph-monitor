# Telegraf Bridge

A simple `rclpy`-based node `telegraf_bridge`.

This currently does:
* Open a UDP socket to a running Telegraf instance
* Instantiate a set of collector plugins to periodically collect Metrics and publish to Telegraf, formatted as Influx line format
* Collectors implemented:
  * Subscriber to `/topic_statistics` to send TopicStatistic

## Configuration

See [params_decl.yaml](./telegraf_bridge/params_decl.yaml) for full parameters declaration for the node.

## Usage

```
ros2 run telegraf_bridge telegraf_bridge
```
