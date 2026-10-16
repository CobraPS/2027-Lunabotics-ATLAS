"""Monitor the NetworkTables connection to the RoboRIO."""
import math

from geometry_msgs.msg import Twist
import ntcore
import rclpy
from rclpy.node import Node


class RoboRIOBridge(Node):
    """Report changes in NetworkTables connection state from a ROS 2 node."""

    def __init__(self):
        """Set up the NetworkTables client and connection timer."""
        super().__init__('roborio_bridge')

        self.declare_parameter('nt_server', '')
        self.nt_server = self.get_parameter('nt_server').value

        self.nt_instance = ntcore.NetworkTableInstance.getDefault()
        self.nt_instance.startClient4('atlas-jetson')

        if self.nt_server:
            self.nt_instance.setServer(self.nt_server)
            self.get_logger().info(
                f'NetworkTables client configured for server: {self.nt_server}'
            )
        else:
            self.get_logger().warning(
                'No NetworkTables server configured.'
            )

        self.nt_connected = None
        self.connection_timer = self.create_timer(
            1.0,
            self.check_nt_connection,
        )

        self.linear_x = 0.0
        self.angular_z = 0.0
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10,
        )

    def check_nt_connection(self):
        """Log a message when the connection state changes."""
        connected = self.nt_instance.isConnected()

        if connected == self.nt_connected:
            return

        self.nt_connected = connected

        if connected:
            self.get_logger().info(
                'Connected to NetworkTables server.'
            )
        elif self.nt_server:
            self.get_logger().warning(
                f'Not connected to NetworkTables server: {self.nt_server}'
            )

    def cmd_vel_callback(self, msg):
        """Store the latest drive command received on /cmd_vel."""
        linear_x = msg.linear.x
        angular_z = msg.angular.z

        if not (math.isfinite(linear_x) and math.isfinite(angular_z)):
            self.get_logger().warning(
                'Ignoring /cmd_vel message with non-finite values.'
            )
            return

        self.linear_x = linear_x
        self.angular_z = angular_z
        self.get_logger().info(
            f'Drive command: linear.x={linear_x:.2f}, '
            f'angular.z={angular_z:.2f}'
        )


def main(args=None):
    """Run the RoboRIO bridge node until interrupted."""
    rclpy.init(args=args)

    node = RoboRIOBridge()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()