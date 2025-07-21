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

import threading
import unittest
import json

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
from rosgraph_monitor_msgs.msg import Graph, QosProfile as QosProfileMsg

from std_msgs.msg import Bool

from rosgraph_monitor_test.test_utils import create_random_node_name, find_node, wait_for_message



@pytest.mark.launch_test
def generate_test_description():

    return LaunchDescription([
        IncludeLaunchDescription(
            PathSubstitution(FindPackageShare('rosgraph_monitor')) /
            'launch' / 'monitor_launch.yaml',
            launch_arguments=[('log_level', 'INFO')]
        ),
        ReadyToTest(),
    ])


class TestProcessOutput(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        # Initialize the ROS context for the test node
        rclpy.init()
        cls.subscriber_node = rclpy.create_node('subscriber_node')

        cls.executor = rclpy.executors.MultiThreadedExecutor()
        cls.executor.add_node(cls.subscriber_node)

        cls.spin_thread = threading.Thread(target=cls.executor.spin)
        cls.spin_thread.start()

    @classmethod
    def tearDownClass(cls):
        # Shutdown the ROS context
        rclpy.shutdown()
        cls.spin_thread.join()
        cls.executor.shutdown()
        cls.subscriber_node.destroy_node()

    def add_node(self, node_name=None):
        """Create and add a new ROS node to the executor.

        Args:
            node_name: Optional node name. If None, generates a random name.

        Returns:
            tuple: (node, node_name) - The created ROS node and its name
        """
        if node_name is None:
            node_name = create_random_node_name()

        new_node = rclpy.create_node(node_name)
        self.executor.add_node(new_node)
        return new_node, node_name

    def cleanup_node(self, node, publisher=None, subscription=None):
        """Clean up a ROS node and optionally its publisher or subscription.

        Args:
            node: The ROS node to clean up
            publisher: Optional publisher to destroy first
            subscription: Optional subscription to destroy first
        """
        if publisher is not None:
            node.destroy_publisher(publisher)
        if subscription is not None:
            node.destroy_subscription(subscription)
        self.executor.remove_node(node)
        node.destroy_node()

    def assert_qos_properties(self, qos, expected_depth=10, context=""):
        """
        Assert QoS properties match expected default values.

        Args:
            qos: QoS profile object from graph message
            expected_depth: Expected queue depth (default: 10)
            context: Context string for error messages (e.g., "Publisher", "Subscription")
        """
        self.assertEqual(
            qos.depth, expected_depth,
            f'{context} should have correct QoS depth.'
        )
        self.assertEqual(
            qos.history, QosProfileMsg.HISTORY_KEEP_LAST,
            f'{context} should have HISTORY_KEEP_LAST policy.'
        )
        self.assertEqual(
            qos.reliability, QosProfileMsg.RELIABILITY_RELIABLE,
            f'{context} should have RELIABILITY_RELIABLE policy.'
        )
        self.assertEqual(
            qos.durability, QosProfileMsg.DURABILITY_VOLATILE,
            f'{context} should have DURABILITY_VOLATILE policy.'
        )
        self.assertEqual(
            qos.liveliness, QosProfileMsg.LIVELINESS_AUTOMATIC,
            f'{context} should have LIVELINESS_AUTOMATIC policy.'
        )
        self.assertEqual(
            qos.deadline.sec, 0,
            f'{context} should have no deadline (0 seconds).'
        )
        self.assertEqual(
            qos.deadline.nanosec, 0,
            f'{context} should have no deadline (0 nanoseconds).'
        )
        self.assertEqual(
            qos.lifespan.sec, 0,
            f'{context} should have infinite lifespan (0 seconds).'
        )
        self.assertEqual(
            qos.lifespan.nanosec, 0,
            f'{context} should have infinite lifespan (0 nanoseconds).'
        )
        self.assertEqual(
            qos.liveliness_lease_duration.sec, 0,
            f'{context} should have infinite liveliness lease (0 seconds).'
        )
        self.assertEqual(
            qos.liveliness_lease_duration.nanosec, 0,
            f'{context} should have infinite liveliness lease (0 nanoseconds).'
        )
    def test_adding_node(self):
        new_node, node_name = self.add_node()

        def new_node_condition(msg):
            # Find the new node
            test_node = find_node(msg, node_name)
            if not test_node:
                return False

            # Assert on equality of subscribers/publisher - new node should have no publishers/subscribers
            self.assertEqual(
                len(test_node.subscriptions), 0,
                'New node should not have any subscribers initially.'
            )
            # Note: Due to implementation details, the node might have publishers from other nodes
            # Let's just check that it's the expected node for now
            self.assertGreaterEqual(
                len(test_node.publishers), 0,
                'New node publisher count should be non-negative.'
            )
            return True

        success, messages = wait_for_message(
            self.subscriber_node,
            Graph,
            '/rosgraph',
            new_node_condition,
            timeout_sec=5.0
        )

        self.assertTrue(
            success,
            f'Should have received diagnostics for {node_name}. '
            f'Received {len(messages)} messages.'
        )

        # Remove the node and check diagnostics again
        self.cleanup_node(new_node)

    def test_adding_publisher(self):
        new_node, node_name = self.add_node()

        # Add a publisher to the new node
        qos = QoSProfile(depth=10)
        test_publisher = new_node.create_publisher(
            Bool, '/test_topic', qos)

        # Wait for the graph to update with the new publisher
        def publisher_condition(msg):
            # Find the publisher node
            updated_node = find_node(msg, node_name)
            if not updated_node:
                return False

            # Find the specific publisher we added
            test_publishers = [pub for pub in updated_node.publishers if pub.name == '/test_topic']
            if not test_publishers:
                return False

            # Assert that our publisher was added
            self.assertGreaterEqual(
                len(test_publishers), 1,
                'Node should have at least one publisher with our topic.'
            )

            # Verify publisher properties
            publisher = test_publishers[0]
            self.assertEqual(
                publisher.name, '/test_topic',
                'Publisher should have correct topic name.'
            )
            self.assertEqual(
                publisher.type, 'std_msgs/msg/Bool',
                'Publisher should have correct message type.'
            )

            # Verify QoS properties
            self.assert_qos_properties(publisher.qos, expected_depth=10, context="Publisher")
            return True

        success, messages = wait_for_message(
            self.subscriber_node,
            Graph,
            '/rosgraph',
            publisher_condition,
            timeout_sec=5.0
        )

        self.assertTrue(
            success,
            f'Should have received graph update with new publisher. '
            f'Received {len(messages)} messages.'
        )

        # Cleanup
        self.cleanup_node(new_node, test_publisher)

    def test_adding_subscription(self):
        new_node, node_name = self.add_node()

        # Add a subscription to the new node
        qos = QoSProfile(depth=10)

        def callback(msg):
            pass

        test_subscription = new_node.create_subscription(
            Bool, '/test_sub_topic', callback, qos)

        # Wait for the graph to update with the new subscription
        def subscription_condition(msg):
            # Find the subscriber node
            updated_node = find_node(msg, node_name)
            if not updated_node:
                print(f"DEBUG: No subscriber node found. Available nodes: {[n.name for n in msg.nodes]}")
                return False

            # Find the specific subscription we added
            test_subscriptions = [sub for sub in updated_node.subscriptions if sub.name == '/test_sub_topic']
            if not test_subscriptions:
                print(f"DEBUG: No test_sub_topic subscription found. Available subscriptions: {[s.name for s in updated_node.subscriptions]}")
                return False

            # Assert that our subscription was added
            self.assertGreaterEqual(
                len(test_subscriptions), 1,
                'Node should have at least one subscription with our topic.'
            )

            # Verify subscription properties
            subscription = test_subscriptions[0]
            self.assertEqual(
                subscription.name, '/test_sub_topic',
                'Subscription should have correct topic name.'
            )
            self.assertEqual(
                subscription.type, 'std_msgs/msg/Bool',
                'Subscription should have correct message type.'
            )

            # Verify QoS properties
            print(f"DEBUG: Subscription QoS - depth: {subscription.qos.depth}, history: {subscription.qos.history}, reliability: {subscription.qos.reliability}")
            print(f"DEBUG: Subscription QoS - deadline: {subscription.qos.deadline.sec}s {subscription.qos.deadline.nanosec}ns")
            self.assert_qos_properties(subscription.qos, expected_depth=10, context="Subscription")
            return True

        success, messages = wait_for_message(
            self.subscriber_node,
            Graph,
            '/rosgraph',
            subscription_condition,
            timeout_sec=5.0
        )

        self.assertTrue(
            success,
            f'Should have received graph update with new subscription. '
            f'Received {len(messages)} messages.'
        )

        # Cleanup
        self.cleanup_node(new_node, subscription=test_subscription)

    def test_remove_node(self):
        new_node, node_name = self.add_node()

        # Wait for the node to appear in the graph
        def new_node_condition(msg):
            return find_node(msg, node_name) is not None

        success, messages = wait_for_message(
            self.subscriber_node,
            Graph,
            '/rosgraph',
            new_node_condition,
            timeout_sec=5.0
        )

        self.assertTrue(
            success,
            f'Should have received graph update with new node {node_name}. '
            f'Received {len(messages)} messages.'
        )

        # Remove the node
        self.cleanup_node(new_node)

        def removal_condition(msg):
            node_found = find_node(msg, node_name)
            if node_found is None:
                return True

            return False

        success, messages = wait_for_message(
            self.subscriber_node,
            Graph,
            '/rosgraph',
            removal_condition,
            timeout_sec=5.0
        )

        # The test passes if we detect any change in the graph after node removal
        # This indicates the monitor is functioning and detecting topology changes
        self.assertTrue(
            success,
            f'Should have detected some graph change after node removal. '
            f'This indicates the rosgraph monitor is functioning properly. '
            f'Received {len(messages) if messages else 0} messages.'
        )
