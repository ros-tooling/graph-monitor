# SPDX-FileCopyrightText: 2026 Polymath Robotics, Inc.
# SPDX-License-Identifier: Apache-2.0
"""Run the shared memory statistics collector. One of these per machine."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    poll_period_ms = LaunchConfiguration('poll_period_ms')
    output_topic = LaunchConfiguration('output_topic')

    return LaunchDescription([
        DeclareLaunchArgument(
            'poll_period_ms',
            default_value='200',
            description=(
                'How often to drain every segment. Must be fast enough that a writer does not '
                'lap its ring between polls, or samples are lost.'
            ),
        ),
        DeclareLaunchArgument(
            'output_topic',
            default_value='/topic_statistics',
            description='Topic to republish collected statistics on.',
        ),
        Node(
            package='topic_stats_collector',
            executable='topic_stats_collector_node',
            name='topic_stats_collector',
            output='screen',
            parameters=[
                {
                    'poll_period_ms': poll_period_ms,
                    'output_topic': output_topic,
                }
            ],
        ),
    ])
