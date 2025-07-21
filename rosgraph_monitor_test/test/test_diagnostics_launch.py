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
from std_msgs.msg import Bool

from rosgraph_monitor_test.test_utils import wait_for_message


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
        cls.publisher_node = rclpy.create_node('publisher_node')
        cls.subscriber_node = rclpy.create_node('subscriber_node')

        cls.executor = rclpy.executors.MultiThreadedExecutor()
        cls.executor.add_node(cls.publisher_node)
        cls.executor.add_node(cls.subscriber_node)

        # Configure QoS based on RMW implementation
        if os.environ.get('RMW_IMPLEMENTATION_WRAPPER') == 'rmw_stats_shim':
            qos = QoSProfile(depth=10, deadline=Duration(seconds=0.1))
        else:
            qos = QoSProfile(depth=10)

        # Create publisher and timer to generate activity
        cls.dummy_publisher = cls.publisher_node.create_publisher(Bool, '/bool_publisher', qos)
        cls.publish_timer = cls.publisher_node.create_timer(
            timer_period_sec=0.1, callback=cls.publisher_callback)

        cls.spin_thread = threading.Thread(target=cls.executor.spin)
        cls.spin_thread.start()

    @classmethod
    def publisher_callback(cls):
        msg = Bool()
        msg.data = True
        cls.dummy_publisher.publish(msg)

    @classmethod
    def tearDownClass(cls):
        # Shutdown the ROS context
        cls.executor.shutdown()
        cls.spin_thread.join()
        cls.subscriber_node.destroy_node()
        cls.publisher_node.destroy_node()
        rclpy.shutdown()

    def test_diagnostics(self):
        # Wait for diagnostic message with all OK statuses
        def diagnostic_condition(msg):
            return (len(msg.status) > 0 and
                    all(status.level == DiagnosticStatus.OK for status in msg.status))

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
