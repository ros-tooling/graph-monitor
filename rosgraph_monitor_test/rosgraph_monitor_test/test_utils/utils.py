# SPDX-FileCopyrightText: 2025 Polymath Robotics, Inc.
# SPDX-License-Identifier: Apache-2.0
import time
import uuid

import rclpy


def create_random_node_name():
    """Generate a random node name for testing."""
    return f'test_node_{uuid.uuid4().hex}'


def find_node(graph_msg, node_name):
    """
    Find a node in the graph message by name.

    Args:
        graph_msg: Graph message from rosgraph_monitor
        node_name: Name of the node to find (with or without leading '/')

    Returns
    -------
        Node object if found, None otherwise

    """
    # Ensure node_name starts with '/'
    if not node_name.startswith('/'):
        node_name = f'/{node_name}'

    for node in graph_msg.nodes:
        if node.name == node_name:
            return node
    return None


def wait_for_message_sync(node, message_type, topic, condition_func, timeout_sec=5.0):
    """
    Wait for a message that meets a condition or timeout.

    Args:
        node: ROS2 node to use for spinning
        message_type: The message type to subscribe to
        topic: Topic name to listen on
        condition_func: Function that takes a message and returns True if condition is met
        timeout_sec: Maximum time to wait in seconds

    Returns
    -------
        tuple: (success: bool, messages: list) - success indicates if condition was met

    """
    messages = []

    def callback(msg):
        messages.append(msg)

    subscriber = node.create_subscription(
        message_type,
        topic,
        callback,
        1,  # QoS depth
    )

    # Create separate executor for this operation to avoid race condition
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(node)

    start_time = time.time()
    end_time = start_time + timeout_sec

    try:
        while time.time() < end_time:
            executor.spin_once(timeout_sec=0.1)

            # Check if any message meets the condition
            if messages and condition_func(messages[-1]):
                return True, messages

        # Timeout reached without meeting condition
        return False, messages

    finally:
        executor.remove_node(node)
        node.destroy_subscription(subscriber)
