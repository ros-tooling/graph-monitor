# SPDX-FileCopyrightText: 2025 Polymath Robotics, Inc.
# SPDX-License-Identifier: Apache-2.0
import os
import threading
import unittest

import pytest
import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.substitutions import PathSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_testing.actions import ReadyToTest
from rclpy.duration import Duration
from rclpy.qos import QoSProfile
from std_msgs.msg import Bool

from rosgraph_monitor_test.test_utils import MessageCollector


@pytest.mark.launch_test
def generate_test_description():
    return LaunchDescription([
        IncludeLaunchDescription(
            PathSubstitution(FindPackageShare('rosgraph_monitor')) / 'launch' / 'monitor_launch.yaml',
            launch_arguments=[('log_level', 'DEBUG')],
        ),
        ReadyToTest(),
    ])


class TestProcessOutput(unittest.TestCase):
    def setUp(self):
        # Initialize the ROS context for the test node
        rclpy.init()
        self.publisher_node = rclpy.create_node('publisher_node')
        self.subscriber_node = rclpy.create_node('subscriber_node')

        self.executor = rclpy.executors.MultiThreadedExecutor()
        self.executor.add_node(self.publisher_node)
        self.executor.add_node(self.subscriber_node)

        # Configure QoS based on RMW implementation
        if os.environ.get('RMW_IMPLEMENTATION_WRAPPER') == 'rmw_stats_shim':
            qos = QoSProfile(depth=10, deadline=Duration(seconds=0.1))
        else:
            qos = QoSProfile(depth=10)

        # Create publisher and timer to generate activity
        self.dummy_publisher = self.publisher_node.create_publisher(Bool, '/bool_publisher', qos)
        self.publish_timer = self.publisher_node.create_timer(timer_period_sec=0.1, callback=self.publisher_callback)

        self.spin_thread = threading.Thread(target=self.executor.spin)
        self.spin_thread.start()

    def publisher_callback(self):
        msg = Bool()
        msg.data = True
        self.dummy_publisher.publish(msg)

    def tearDown(self):
        # Shutdown the ROS context
        self.executor.shutdown()
        self.spin_thread.join()
        self.subscriber_node.destroy_node()
        self.publisher_node.destroy_node()
        rclpy.shutdown()

    def test_diagnostics(self):
        # Wait for diagnostic message with all OK statuses
        def diagnostic_condition(msg):
            return len(msg.status) > 0 and all(status.level == DiagnosticStatus.OK for status in msg.status)

        # The aggregator publishes periodically rather than on change,
        # so this test does not need to arm the collector ahead of an action.
        with MessageCollector(self.subscriber_node, self.executor, DiagnosticArray, '/diagnostics_agg') as collector:
            success, messages = collector.wait_until(diagnostic_condition, timeout_sec=5.0)

        self.assertTrue(
            success,
            f'Should have received at least one /diagnostics_agg message with all OK statuses. '
            f'Received {len(messages)} messages.',
        )
