"""Launch the ZED 2i camera using the official ROS 2 wrapper."""
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Create the standalone ATLAS ZED 2i launch description."""
    zed_camera_launch_file = PathJoinSubstitution([
        FindPackageShare('zed_wrapper'),
        'launch',
        'zed_camera.launch.py'
    ])

    include_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(zed_camera_launch_file),
        launch_arguments={'camera_model': 'zed2i'}.items()
    )

    return LaunchDescription([
        include_launch
    ])
