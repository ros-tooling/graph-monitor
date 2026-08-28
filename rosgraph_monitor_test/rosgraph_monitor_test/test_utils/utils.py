# SPDX-FileCopyrightText: 2025 Polymath Robotics, Inc.
# SPDX-License-Identifier: Apache-2.0
import threading
import time
import uuid


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


def first_matching(messages, condition):
    """
    Return the first message that satisfies condition, or None if no message does.

    Args:
        messages: Iterable of messages, oldest first
        condition: Predicate taking a message and returning a bool

    Returns
    -------
        The first matching message, or None

    """
    return next((msg for msg in messages if condition(msg)), None)


class MessageCollector:
    """
    Collect every message published on a topic while the collector is entered.

    The subscription lives for the lifetime of the context manager, so a collector entered
    before an action observes all messages that the action causes.
    Topics that publish only on change require this ordering:
    a subscription created after the change misses the message describing it.

    The node must already be spun by an executor the caller owns.
    This collector creates no executor and spins nothing,
    so it cannot race the caller's executor for callbacks.

    Example
    -------
        with MessageCollector(node, executor, Graph, '/rosgraph') as collector:
            collector.wait_for_any()
            do_something_that_changes_the_graph()
            success, messages = collector.wait_until(lambda msg: len(msg.nodes) > 3)

    """

    def __init__(self, node, executor, message_type, topic, qos_depth=10):
        """
        Configure a collector without subscribing.

        Args:
            node: ROS 2 node that a caller-owned executor is already spinning
            executor: The executor spinning the node, woken when the subscription set changes
            message_type: The message type to subscribe to
            topic: Topic name to listen on
            qos_depth: Subscription queue depth, deep enough to hold a burst of messages

        """
        self._node = node
        self._executor = executor
        self._message_type = message_type
        self._topic = topic
        self._qos_depth = qos_depth
        self._subscription = None
        # Guards _messages and _checked, and wakes waiters when a message arrives.
        self._arrival = threading.Condition()
        self._messages = []
        # Number of leading messages that wait_until has already passed to a condition.
        self._checked = 0

    def __enter__(self):
        """Create the subscription and start collecting."""
        self._subscription = self._node.create_subscription(
            self._message_type,
            self._topic,
            self._collect,
            self._qos_depth,
        )
        # Waking the executor makes it rebuild its wait set, so it notices the new subscription immediately.
        self._executor.wake()
        return self

    def __exit__(self, exc_type, exc_value, traceback):
        """Destroy the subscription and stop collecting."""
        self._node.destroy_subscription(self._subscription)
        self._executor.wake()
        self._subscription = None
        return False

    def _collect(self, msg):
        with self._arrival:
            self._messages.append(msg)
            self._arrival.notify_all()

    @property
    def messages(self):
        """Every message collected so far, oldest first."""
        with self._arrival:
            return list(self._messages)

    def wait_for_any(self, timeout_sec=5.0):
        """
        Block until at least one message has been collected.

        A publisher that publishes on graph change publishes because this collector subscribed,
        so receiving that message proves the publisher-to-subscriber path is live
        before the caller acts.

        Args:
            timeout_sec: Maximum time to wait in seconds

        Returns
        -------
            True if a message arrived, False on timeout

        """
        with self._arrival:
            return self._arrival.wait_for(lambda: bool(self._messages), timeout=timeout_sec)

    def wait_until(self, condition, timeout_sec=5.0):
        """
        Block until condition holds for a collected message, or until timeout.

        Every message not yet passed to a condition is checked, not just the most recent one,
        so a match is never lost to a later message overwriting it.
        Condition must be a pure predicate: it is called on partial and unrelated messages,
        and returning False means keep waiting.

        Args:
            condition: Predicate taking a message and returning a bool
            timeout_sec: Maximum time to wait in seconds

        Returns
        -------
            tuple: (success: bool, messages: list) - messages holds everything collected so far

        """
        deadline = time.monotonic() + timeout_sec
        while True:
            with self._arrival:
                start = self._checked
                unchecked = self._messages[start:]

            # Condition is caller code, so it runs outside the lock.
            # A match consumes the messages up to and including itself, and no more,
            # so a later call sees everything that followed the match.
            for offset, msg in enumerate(unchecked):
                if condition(msg):
                    with self._arrival:
                        self._checked = start + offset + 1
                    return True, self.messages

            with self._arrival:
                self._checked = start + len(unchecked)
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    return False, self.messages
                self._arrival.wait_for(lambda: len(self._messages) > self._checked, timeout=remaining)
