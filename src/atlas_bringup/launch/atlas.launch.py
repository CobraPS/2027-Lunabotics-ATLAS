"""Launch the currently available ATLAS ROS 2 components."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Create the ATLAS launch description."""
    nt_server = LaunchConfiguration('nt_server')

    return LaunchDescription([
        DeclareLaunchArgument(
            'nt_server',
            default_value='',
            description='NetworkTables server address for the RoboRIO.',
        ),
        Node(
            package='atlas_roborio_bridge',
            executable='roborio_bridge',
            name='roborio_bridge',
            output='screen',
            parameters=[{
                'nt_server': nt_server,
            }],
        ),
    ])
