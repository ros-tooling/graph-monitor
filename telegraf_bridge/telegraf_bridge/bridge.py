# Copyright 2024 Bonsai Robotics, Inc - All Rights Reserved
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import socket

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rosgraph_monitor_msgs.msg import TopicStatistics

from telegraf_bridge.generated_parameters import telegraf_bridge as generated_parameters

from .collectors.topic_statistics import TopicStatisticsCollector


def format_influx_value(value):
    if isinstance(value, int):
        return f'{value}i'
    elif isinstance(value, float):
        return f'{value}'
    elif isinstance(value, str):
        return value
    else:
        raise TypeError(f'Unknown type "{type(value)}" for influx line protocol.')


def influx_line_format(measurement, *, fields, tags={}, timestamp=None):
    tag_str = ''
    if tags:
        tag_str = ',' + ','.join([f'{k}={v}' for k, v in tags.items()])

    field_str = ','.join([f'{k}={format_influx_value(v)}' for k, v in fields.items()])
    ts_str = '' if timestamp is None else f' {int(timestamp * 1e9)}'
    return f'{measurement}{tag_str} {field_str}{ts_str}'


class TelegrafBridgeNode(Node):

    def __init__(self):
        super().__init__('telegraf_collector')

        self.param_listener = generated_parameters.ParamListener(self)
        self.params = self.param_listener.get_params()
        self.get_logger().info(
            f'Sending to Telegraf at {self.params.telegraf_uri}:{self.params.telegraf_port} '
            f'every {self.params.publish_interval_s} seconds'
        )
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        # Configure metric collection inputs
        self.collectors = []

        topic_stats = TopicStatisticsCollector(now_fn=self.get_clock().now)
        self.sub_stats = self.create_subscription(
            TopicStatistics,
            '/topic_statistics',
            topic_stats.add_msg,
            10,
        )
        self.collectors.append(topic_stats)

        self.pub_timer = self.create_timer(self.params.publish_interval_s, self.publish_metrics)

    def publish_metrics(self):
        measurements = []
        for collector in self.collectors:
            measurements.extend(collector.get_measurements())
        for measurement in measurements:
            msg = influx_line_format(
                measurement=measurement.name,
                tags=measurement.tags,
                fields=measurement.fields,
            )
            self.sock.sendto(
                msg.encode(), (self.params.telegraf_uri, self.params.telegraf_port)
            )

    def destroy_node(self):
        for collector in self.collectors:
            collector.stop()
        super().destroy_node()


def main():
    rclpy.init()
    node = TelegrafBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except ExternalShutdownException:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
