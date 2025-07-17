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
        self.diagnostics = []
        self.diagnostics_agg_msgs = []
        self.topic_statistics = []
        self.rosgraph_msgs = []
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
        sub = self.subscriber_node.create_subscription(
            DiagnosticArray,
            '/diagnostics_agg',
            lambda msg: self.diagnostics_agg_msgs.append(msg),
            QoSProfile(depth=1),
        )

        end_time = time.time() + 5
        while time.time() < end_time:
            rclpy.spin_once(self.publisher_node, timeout_sec=0.1)

        self.assertGreater(
            len(self.diagnostics_agg_msgs),
            0,
            'Should have received at least one /diagnostics_agg message',
        )

        last_msg = self.diagnostics_agg_msgs.pop(-1)
        self.assertTrue(
            all(status.level == DiagnosticStatus.OK for status in last_msg.status),
            f'All diagnostic statuses should be healthy: {last_msg}',
        )

        self.subscriber_node.destroy_subscription(sub)
        self.assertGreater(
            len(self.diagnostics_agg_msgs),
            0,
            'Should have received at least one more /diagnostics_agg message',
        )

        last_msg = self.diagnostics_agg_msgs.pop(-1)
        self.assertTrue(
            all(status.level == DiagnosticStatus.OK for status in last_msg.status),
            f'All diagnostic statuses should be healthy: {last_msg}',
        )

    def test_rosgraph_messages(self):
        rosgraph_sub = self.subscriber_node.create_subscription(
            Graph,
            '/rosgraph',
            lambda msg: self.rosgraph_msgs.append(msg),
            1)

        end_time = time.time() + 5
        while time.time() < end_time:
            rclpy.spin_once(self.publisher_node, timeout_sec=0.1)

        self.assertGreater(
            len(self.rosgraph_msgs),
            0, 'There should be at least one /rosgraph message')

        last_msg = self.rosgraph_msgs[-1]
        self.assertIsNotNone(
            last_msg, 'Last rosgraph message should not be None')

        self.assertTrue(any(
            node.name.startswith('/publisher_node')
            for node in last_msg.nodes),
            'Node info should contain publisher_node details')

        self.subscriber_node.destroy_subscription(rosgraph_sub)
