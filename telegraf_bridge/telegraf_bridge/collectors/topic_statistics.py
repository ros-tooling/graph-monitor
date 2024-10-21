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

from typing import List

from rclpy.duration import Duration
from rclpy.time import Time
from rosgraph_monitor_msgs.msg import TopicStatistic, TopicStatistics

from telegraf_bridge.collector import Measurement, MetricCollector


def _statistic_type_to_str(statistic_type):
    return {
        TopicStatistic.PUBLISHED_PERIOD: 'published_period',
        TopicStatistic.RECEIVED_PERIOD: 'received_period',
        TopicStatistic.TAKE_AGE: 'take_age',
    }[statistic_type]


class StampedStatistic:

    def __init__(self, timestamp: Time, stat: TopicStatistic):
        self.timestamp = timestamp
        self.stat = stat


class TopicStatisticsAggregator:
    """Aggregates many TopicStatistics messages into a single message, pruning old data points."""

    def __init__(self, stale_timeout=Duration(seconds=2)):
        self.stale_timeout = stale_timeout
        self.stats = {}

    def add(self, msg: TopicStatistics):
        for stat in msg.statistics:
            key = (stat.statistic_type, stat.node_name, stat.topic_name)
            self.stats[key] = StampedStatistic(Time.from_msg(msg.timestamp), stat)

    def aggregate(self, time: Time) -> TopicStatistics:
        """Given all messages added so far, create aggregate message, pruning stale entries."""
        # Filter out stale statistics
        self.stats = {
            k: v
            for k, v in self.stats.items()
            if (time - v.timestamp) < self.stale_timeout
        }
        msg = TopicStatistics()
        msg.timestamp = time.to_msg()
        msg.statistics = [v.stat for v in self.stats.values()]
        return msg


class TopicStatisticsCollector(MetricCollector):

    def __init__(self, *, now_fn):
        self.stats_agg = TopicStatisticsAggregator()
        self.now_fn = now_fn

    def get_measurements(self) -> List[Measurement]:
        topic_stats = self.stats_agg.aggregate(self.now_fn())
        return [
            Measurement(
                name='topic_statistics',
                tags={
                    'node': stat.node_name,
                    'topic': stat.topic_name,
                    'type': _statistic_type_to_str(stat.statistic_type),
                },
                fields={
                    'mean': Duration.from_msg(stat.mean).nanoseconds,
                    'min': Duration.from_msg(stat.min).nanoseconds,
                    'max': Duration.from_msg(stat.max).nanoseconds,
                    'window_count': stat.window_count,
                },
            )
            for stat in topic_stats.statistics
        ]

    def add_msg(self, msg: TopicStatistics):
        self.stats_agg.add(msg)

    def stop(self):
        pass
