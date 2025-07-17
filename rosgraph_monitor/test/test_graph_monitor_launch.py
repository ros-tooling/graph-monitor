#!/usr/bin/env python3

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

import os
import threading
import time
import unittest

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.substitutions import PathSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_testing.actions import ReadyToTest
import pytest
import rclpy
from rclpy.duration import Duration
from rclpy.qos import QoSProfile
from rosgraph_monitor_msgs.msg import Graph
from std_msgs.msg import Bool


def wait_for_message(node, message_type, topic, condition_func, timeout_sec=5.0):
    """
    Utility function to wait for a message that meets a condition or timeout.

    Args:
        node: ROS2 node to use for spinning
        message_type: The message type to subscribe to
        topic: Topic name to listen on
        condition_func: Function that takes a message and returns True if condition is met
        timeout_sec: Maximum time to wait in seconds

    Returns:
        tuple: (success: bool, messages: list) - success indicates if condition was met
    """
    messages = []

    def callback(msg):
        messages.append(msg)

    subscriber = node.create_subscription(
        message_type,
        topic,
        callback,
        1  # QoS depth
    )

    start_time = time.time()
    end_time = start_time + timeout_sec

    try:
        while time.time() < end_time:
            rclpy.spin_once(node, timeout_sec=0.1)

            # Check if any message meets the condition
            if messages and condition_func(messages[-1]):
                return True, messages

        # Timeout reached without meeting condition
        return False, messages

    finally:
        node.destroy_subscription(subscriber)


@pytest.mark.launch_test
def generate_test_description():
    return LaunchDescription([
        IncludeLaunchDescription(
            PathSubstitution(FindPackageShare('rosgraph_monitor')) /
            'launch' / 'monitor_launch.yaml',
            launch_arguments=[('log_level', 'DEBUG')]
        ),
        ReadyToTest(),
    ])


class TestProcessOutput(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        # Initialize the ROS context for the test node
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        # Shutdown the ROS context
        rclpy.shutdown()

    def setUp(self):
        # Create a ROS node for tests
        self.publisher_node = rclpy.create_node('publisher_node')
        self.subscriber_node = rclpy.create_node('subscriber_node')

        self.executor = rclpy.executors.MultiThreadedExecutor()
        self.executor.add_node(self.publisher_node)
        self.executor.add_node(self.subscriber_node)

        if os.environ.get('RMW_IMPLEMENTATION_WRAPPER') == 'rmw_stats_shim':
            qos = QoSProfile(depth=10, deadline=Duration(seconds=0.1))
        else:
            qos = QoSProfile(depth=10)

        self.dummy_publisher = self.publisher_node.create_publisher(
            Bool, '/bool_publisher', qos)
        self.publish_timer = self.publisher_node.create_timer(
            timer_period_sec=0.1, callback=self.publisher_callback)

        self.spin_thread = threading.Thread(target=self.executor.spin)
        self.spin_thread.start()

    def publisher_callback(self):
        msg = Bool()
        msg.data = True
        self.dummy_publisher.publish(msg)

    def tearDown(self):
        self.executor.shutdown()
        self.spin_thread.join()
        self.subscriber_node.destroy_node()
        self.publisher_node.destroy_node()

    def test_diagnostics(self):
        # Wait for diagnostic message with all OK statuses
        def diagnostic_condition(msg):
            return len(msg.status) > 0 and all(status.level == DiagnosticStatus.OK for status in msg.status)

        success, messages = wait_for_message(
            self.subscriber_node,
            DiagnosticArray,
            '/diagnostics_agg',
            diagnostic_condition,
            timeout_sec=5.0
        )

        self.assertTrue(
            success,
            f'Should have received at least one /diagnostics_agg message with all OK statuses. '
            f'Received {len(messages)} messages.'
        )

    def test_rosgraph_messages(self):
        # Wait for rosgraph message that contains our publisher node
        def rosgraph_condition(msg):
            return (msg is not None and
                   len(msg.nodes) > 0 and
                   any(node.name.startswith('/publisher_node') for node in msg.nodes))

        success, messages = wait_for_message(
            self.subscriber_node,
            Graph,
            '/rosgraph',
            rosgraph_condition,
            timeout_sec=5.0
        )

        self.assertTrue(
            success,
            f'Should have received at least one /rosgraph message containing publisher_node. '
            f'Received {len(messages)} messages.'
        )
