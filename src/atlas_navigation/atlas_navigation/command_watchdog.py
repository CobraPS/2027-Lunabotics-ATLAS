"""Expire simulation velocity commands without depending on the simulation clock."""

import math

from geometry_msgs.msg import Twist
from rcl_interfaces.msg import ParameterDescriptor
import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.node import Node


class CommandWatchdog(Node):
    """Forward fresh commands and publish zero when their steady-clock age expires."""

    def __init__(self, **kwargs):
        """Configure the startup-only timeout and start a 50 Hz steady-clock timer."""
        super().__init__('simulation_command_watchdog', **kwargs)
        timeout = self.declare_parameter(
            'cmd_timeout', 0.5,
            ParameterDescriptor(read_only=True, description='Command timeout in wall seconds'),
        ).value
        if not math.isfinite(timeout) or timeout <= 0:
            self.destroy_node()
            raise ValueError('cmd_timeout must be finite and greater than zero')
        self._timeout_ns = timeout * 1e9
        self._steady_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self._last_received_ns = None
        self._command = Twist()
        self._publisher = self.create_publisher(Twist, 'simulation/cmd_vel', 1)
        self._subscription = self.create_subscription(Twist, 'cmd_vel', self._on_command, 1)
        self._timer = self.create_timer(0.02, self._publish_command, clock=self._steady_clock)

    def _on_command(self, command):
        if not all(math.isfinite(value) for vector in (command.linear, command.angular)
                   for value in (vector.x, vector.y, vector.z)):
            return
        self._command = command
        self._last_received_ns = self._steady_clock.now().nanoseconds

    def _publish_command(self):
        fresh = (self._last_received_ns is not None and
                 self._steady_clock.now().nanoseconds - self._last_received_ns < self._timeout_ns)
        self._publisher.publish(self._command if fresh else Twist())


def main(args=None):
    """Run the simulation-only watchdog."""
    rclpy.init(args=args)
    node = None
    try:
        node = CommandWatchdog()
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()
