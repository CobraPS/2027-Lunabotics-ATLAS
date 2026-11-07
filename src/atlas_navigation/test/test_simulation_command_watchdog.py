"""Exercise command expiry through real ROS publishers, timers, and subscriptions."""

import math
import time
import uuid

from geometry_msgs.msg import Twist
import pytest


@pytest.fixture
def ros_pair():
    """Use a unique namespace and a frozen ROS clock to test steady-clock expiry."""
    # Load ROS at execution time so collection does not start native threads
    # before the package's multiprocessing flake8 check.
    from atlas_navigation.command_watchdog import CommandWatchdog
    import rclpy
    from rclpy.context import Context
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node
    from rclpy.parameter import Parameter

    context = Context()
    rclpy.init(context=context)
    namespace = '/watchdog_test_' + uuid.uuid4().hex
    watchdog = CommandWatchdog(
        context=context, namespace=namespace,
        cli_args=['--ros-args', '-r', '/clock:=' + namespace + '/clock'],
        parameter_overrides=[Parameter('cmd_timeout', value=0.15),
                             Parameter('use_sim_time', value=True)],
    )
    peer = Node('peer', context=context, namespace=namespace)
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(watchdog)
    executor.add_node(peer)
    received = []
    publisher = peer.create_publisher(Twist, 'cmd_vel', 1)
    subscription = peer.create_subscription(
        Twist, 'simulation/cmd_vel', lambda msg: received.append((time.monotonic(), msg)), 10)

    def spin_until(predicate, timeout=3):
        deadline = time.monotonic() + timeout
        while not predicate():
            assert time.monotonic() < deadline, 'Timed out waiting for watchdog output'
            executor.spin_once(timeout_sec=0.01)

    spin_until(lambda: publisher.get_subscription_count() == 1 and bool(received))
    try:
        yield watchdog, publisher, received, spin_until
    finally:
        peer.destroy_subscription(subscription)
        executor.shutdown()
        watchdog.destroy_node()
        peer.destroy_node()
        context.shutdown()


def test_timeout_and_repeated_recovery(ros_pair):
    """Start stopped, expire two commands, and recover without a latched timeout."""
    watchdog, publisher, received, spin_until = ros_pair
    assert received[-1][1] == Twist()
    assert watchdog.get_clock().now().nanoseconds == 0
    for speed in (0.25, -0.25):
        command = Twist()
        command.linear.x = speed
        command.angular.z = 0.4
        publisher.publish(command)
        spin_until(lambda: received[-1][1].linear.x == speed)
        accepted_at = received[-1][0]
        spin_until(lambda: received[-1][1] == Twist())
        assert 0.10 <= received[-1][0] - accepted_at < 0.35
    assert watchdog.get_clock().now().nanoseconds == 0


def test_invalid_command_does_not_refresh_timeout(ros_pair):
    """Non-finite input cannot keep a previous movement command alive."""
    _, publisher, received, spin_until = ros_pair
    command = Twist()
    command.linear.x = 0.25
    publisher.publish(command)
    spin_until(lambda: received[-1][1].linear.x == 0.25)
    accepted_at = received[-1][0]
    command.angular.z = math.nan
    while time.monotonic() - accepted_at < 0.25:
        publisher.publish(command)
        previous = len(received)
        spin_until(lambda: len(received) > previous)
    assert received[-1][1] == Twist()


@pytest.mark.parametrize('timeout', [0.0, -0.1, math.nan, math.inf])
def test_reject_invalid_timeout(timeout):
    """A non-positive or non-finite timeout cannot disable expiry accidentally."""
    from atlas_navigation.command_watchdog import CommandWatchdog
    import rclpy
    from rclpy.context import Context
    from rclpy.parameter import Parameter

    context = Context()
    rclpy.init(context=context)
    try:
        with pytest.raises(ValueError, match='finite and greater than zero'):
            CommandWatchdog(context=context, parameter_overrides=[
                Parameter('cmd_timeout', value=timeout)])
    finally:
        context.shutdown()
