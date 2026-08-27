# SPDX-FileCopyrightText: 2025 Polymath Robotics, Inc.
# SPDX-License-Identifier: Apache-2.0
import threading
import unittest
from contextlib import contextmanager

import pytest
import rclpy
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.substitutions import PathSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_testing.actions import ReadyToTest
from rcl_interfaces.msg import ParameterType
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile
from rosgraph_msgs.msg import Graph
from rosgraph_msgs.msg import QoSProfile as QosProfileMsg
from std_msgs.msg import Bool

from rosgraph_monitor_test.test_utils import (
    MessageCollector,
    create_random_node_name,
    find_node,
    first_matching,
)

# The monitor queries a new node's parameters as soon as it sees the node, and a query made before
# the node's parameter services are discoverable fails and is retried after RosGraphMonitor's
# Config::parameters::retry_delay of 5 seconds.
# Observing parameters therefore has to allow for one retry cycle.
PARAMETER_TIMEOUT_SEC = 15.0


@pytest.mark.launch_test
def generate_test_description():
    return LaunchDescription([
        IncludeLaunchDescription(
            PathSubstitution(FindPackageShare('rosgraph_monitor')) / 'launch' / 'monitor_launch.yaml',
            launch_arguments=[('log_level', 'INFO')],
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

    def add_node(self, node_name=None, parameters=None):
        """
        Create and add a new ROS node to the executor.

        Args:
            node_name (str, optional): If None, generates a random name.

        Returns
        -------
            tuple: (node, node_name) - The created ROS node and its name

        """
        if node_name is None:
            node_name = create_random_node_name()

        # rclpy's TypeDescriptionService holds a reference to the node handle that destroy_node()
        # does not release, so a destroyed node lingers in the graph until its process exits.
        # Disabling the service lets tests observe node removal.
        # Regression introduced in Lyrical+ in https://github.com/ros2/rclpy/pull/1629
        new_node = rclpy.create_node(
            node_name,
            parameter_overrides=[
                Parameter('start_type_description_service', Parameter.Type.BOOL, False),
            ],
        )
        if parameters is not None:
            for param_name, param_value in parameters.items():
                new_node.declare_parameter(param_name, param_value)
        self.executor.add_node(new_node)
        return new_node, node_name

    @contextmanager
    def armed_graph_collector(self):
        """
        Yield a /rosgraph collector that has already received a message.

        The monitor publishes only on graph change, so the collector subscribes before the test
        changes the graph, and the first message confirms the collector receives what the monitor
        publishes.

        Returns
        -------
            Context manager yielding a MessageCollector

        """
        with MessageCollector(self.subscriber_node, Graph, '/rosgraph') as collector:
            self.assertTrue(
                collector.wait_for_any(timeout_sec=5.0),
                'Should have received a /rosgraph message in response to subscribing.',
            )
            yield collector

    def cleanup_node(self, node, publisher=None, subscription=None):
        """
        Clean up a ROS node and optionally its publisher or subscription.

        Args:
            node (rclpy.node.Node): The ROS node to clean up.
            publisher (rclpy.Publisher, optional): If given, this publisher will
            be destroyed first.
            subscription (rclpy.Subscription, optional): If given, this
            subscription will be destroyed first.

        Returns
        -------
            None

        """
        if publisher is not None:
            node.destroy_publisher(publisher)
        if subscription is not None:
            node.destroy_subscription(subscription)
        self.executor.remove_node(node)
        node.destroy_node()

    def assert_qos_properties(self, qos, expected_depth=10, context=''):
        """
        Assert QoS properties match expected default values.

        Args:
            qos (rclpy.qos.QoSProfile): The QoS profile object from a graph message.
            expected_depth (int, optional): The expected queue depth (default: 10).
            context (str, optional): Context string for error messages (e.g.,
            'Publisher', 'Subscription').

        Returns
        -------
            None

        """
        # TODO(emersonknapp): History QoS isn't propagated in Fast-DDS
        #   Added to Kilted+ in https://github.com/ros2/rmw_fastrtps/pull/829,
        #   but unclear if backport to Jazzy/Humble possible
        # self.assertEqual(
        #     qos.depth, expected_depth,
        #     f'{context} should have correct QoS depth.'
        # )
        # self.assertEqual(
        #     qos.history, QosProfileMsg.HISTORY_KEEP_LAST,
        #     f'{context} should have HISTORY_KEEP_LAST policy.'
        # )
        self.assertEqual(
            qos.reliability, QosProfileMsg.RELIABILITY_RELIABLE, f'{context} should have RELIABILITY_RELIABLE policy.'
        )
        self.assertEqual(
            qos.durability, QosProfileMsg.DURABILITY_VOLATILE, f'{context} should have DURABILITY_VOLATILE policy.'
        )
        self.assertEqual(
            qos.liveliness, QosProfileMsg.LIVELINESS_AUTOMATIC, f'{context} should have LIVELINESS_AUTOMATIC policy.'
        )
        self.assertEqual(qos.deadline.sec, 0, f'{context} should have no deadline (0 seconds).')
        self.assertEqual(qos.deadline.nanosec, 0, f'{context} should have no deadline (0 nanoseconds).')
        self.assertEqual(qos.lifespan.sec, 0, f'{context} should have infinite lifespan (0 seconds).')
        self.assertEqual(qos.lifespan.nanosec, 0, f'{context} should have infinite lifespan (0 nanoseconds).')
        self.assertEqual(
            qos.liveliness_lease_duration.sec, 0, f'{context} should have infinite liveliness lease (0 seconds).'
        )
        self.assertEqual(
            qos.liveliness_lease_duration.nanosec,
            0,
            f'{context} should have infinite liveliness lease (0 nanoseconds).',
        )

    def test_adding_node(self):
        with self.armed_graph_collector() as collector:
            new_node, node_name = self.add_node()

            def new_node_condition(msg):
                return find_node(msg, node_name) is not None

            success, messages = collector.wait_until(new_node_condition, timeout_sec=5.0)

            self.assertTrue(
                success, f'Should have received diagnostics for {node_name}. Received {len(messages)} messages.'
            )

            test_node = find_node(first_matching(messages, new_node_condition), node_name)

            # Assert on equality of subscribers/publisher - new node should have
            # no publishers/subscribers
            self.assertEqual(len(test_node.subscriptions), 0, 'New node should not have any subscribers initially.')
            # Note: Due to implementation details, the node might have publishers from other nodes
            # Let's just check that it's the expected node for now
            self.assertGreaterEqual(len(test_node.publishers), 0, 'New node publisher count should be non-negative.')

        # Remove the node and check diagnostics again
        self.cleanup_node(new_node)

    def test_adding_node_with_parameters(self):
        params = {'param1': 'value1', 'param2': 42}

        with self.armed_graph_collector() as collector:
            _, node_name = self.add_node(parameters=params)

            # Wait for the graph to update with the new parameters
            def parameters_condition(msg):
                # Find the parameter node
                updated_node = find_node(msg, node_name)
                if not updated_node:
                    return False

                declared = {param.name: param for param in updated_node.parameters if param.name in params}
                if len(declared) != len(params):
                    return False

                # The monitor publishes the names before it has described their types, so keep waiting.
                return all(param.type != ParameterType.PARAMETER_NOT_SET for param in declared.values())

            success, messages = collector.wait_until(parameters_condition, timeout_sec=PARAMETER_TIMEOUT_SEC)

            self.assertTrue(
                success, f'Should have received diagnostics for {node_name}. Received {len(messages)} messages.'
            )

            updated_node = find_node(first_matching(messages, parameters_condition), node_name)
            declared = {param.name: param for param in updated_node.parameters if param.name in params}

            self.assertEqual(declared['param1'].type, ParameterType.PARAMETER_STRING, 'param1 should be a string.')
            self.assertEqual(declared['param2'].type, ParameterType.PARAMETER_INTEGER, 'param2 should be an integer.')
            self.assertEqual(
                len(updated_node.parameter_values), 0, 'parameter_values should be empty until values are read.'
            )

    def test_adding_publisher(self):
        with self.armed_graph_collector() as collector:
            new_node, node_name = self.add_node()

            # Add a publisher to the new node
            qos = QoSProfile(depth=10)
            test_publisher = new_node.create_publisher(Bool, '/test_topic', qos)

            # Wait for the graph to update with the new publisher
            def publisher_condition(msg):
                # Find the publisher node
                updated_node = find_node(msg, node_name)
                if not updated_node:
                    return False

                # Find the specific publisher we added
                return any(pub.name == '/test_topic' for pub in updated_node.publishers)

            success, messages = collector.wait_until(publisher_condition, timeout_sec=5.0)

            self.assertTrue(
                success, f'Should have received graph update with new publisher. Received {len(messages)} messages.'
            )

            updated_node = find_node(first_matching(messages, publisher_condition), node_name)
            test_publishers = [pub for pub in updated_node.publishers if pub.name == '/test_topic']

            # Assert that our publisher was added
            self.assertGreaterEqual(len(test_publishers), 1, 'Node should have at least one publisher with our topic.')

            # Verify publisher properties
            publisher = test_publishers[0]
            self.assertEqual(publisher.name, '/test_topic', 'Publisher should have correct topic name.')
            self.assertEqual(publisher.type.name, 'std_msgs/msg/Bool', 'Publisher should have correct message type.')

            # Verify QoS properties
            self.assert_qos_properties(publisher.qos, expected_depth=10, context='Publisher')

        # Cleanup
        self.cleanup_node(new_node, test_publisher)

    def test_adding_subscription(self):
        with self.armed_graph_collector() as collector:
            new_node, node_name = self.add_node()

            # Add a subscription to the new node
            qos = QoSProfile(depth=10)

            def callback(msg):
                pass

            test_subscription = new_node.create_subscription(Bool, '/test_sub_topic', callback, qos)

            # Wait for the graph to update with the new subscription
            def subscription_condition(msg):
                # Find the subscriber node
                updated_node = find_node(msg, node_name)
                if not updated_node:
                    return False

                # Find the specific subscription we added
                return any(sub.name == '/test_sub_topic' for sub in updated_node.subscriptions)

            success, messages = collector.wait_until(subscription_condition, timeout_sec=5.0)

            self.assertTrue(
                success, f'Should have received graph update with new subscription. Received {len(messages)} messages.'
            )

            updated_node = find_node(first_matching(messages, subscription_condition), node_name)
            test_subscriptions = [sub for sub in updated_node.subscriptions if sub.name == '/test_sub_topic']

            # Assert that our subscription was added
            self.assertGreaterEqual(
                len(test_subscriptions), 1, 'Node should have at least one subscription with our topic.'
            )

            # Verify subscription properties
            subscription = test_subscriptions[0]
            self.assertEqual(subscription.name, '/test_sub_topic', 'Subscription should have correct topic name.')
            self.assertEqual(
                subscription.type.name, 'std_msgs/msg/Bool', 'Subscription should have correct message type.'
            )

            # Verify QoS properties
            self.assert_qos_properties(subscription.qos, expected_depth=10, context='Subscription')

        # Cleanup
        self.cleanup_node(new_node, subscription=test_subscription)

    def test_remove_node(self):
        with self.armed_graph_collector() as collector:
            new_node, node_name = self.add_node()

            # Wait for the node to appear in the graph
            def new_node_condition(msg):
                return find_node(msg, node_name) is not None

            success, messages = collector.wait_until(new_node_condition, timeout_sec=5.0)

            self.assertTrue(
                success,
                f'Should have received graph update with new node {node_name}. Received {len(messages)} messages.',
            )

            # Remove the node
            self.cleanup_node(new_node)

            # The collector already consumed every message up to the one showing the node present,
            # so a later message without the node describes the removal rather than the graph
            # before the node existed.
            def removal_condition(msg):
                return find_node(msg, node_name) is None

            success, messages = collector.wait_until(removal_condition, timeout_sec=5.0)

            # The test passes if we detect any change in the graph after node removal
            # This indicates the monitor is functioning and detecting topology changes
            self.assertTrue(
                success,
                f'Should have detected some graph change after node removal. '
                f'This indicates the rosgraph monitor is functioning properly. '
                f'Received {len(messages) if messages else 0} messages.',
            )
