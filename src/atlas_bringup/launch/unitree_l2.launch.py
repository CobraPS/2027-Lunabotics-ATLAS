from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions.if_condition import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    lidar_ip_arg = DeclareLaunchArgument('lidar_ip', default_value='10.0.9.3')

    lidar_ip = LaunchConfiguration('lidar_ip')

    local_ip_arg = DeclareLaunchArgument(name='local_ip', default_value='10.0.9.1')

    local_ip = LaunchConfiguration('local_ip')

    lidar_node = Node(
        package='unitree_lidar_ros2',
        executable='unitree_lidar_ros2_node',
        name='unitree_lidar_ros2_node',
        output='screen',
        parameters=[
            {'initialize_type': 2},
            {'work_mode': 0},
            {'lidar_ip': lidar_ip},
            {'local_ip': local_ip},
            {'lidar_port': 6101},
            {'local_port': 6201},
        ],
    )
    rviz_config = PathJoinSubstitution([
        FindPackageShare('atlas_bringup'), 'rviz', 'unitree_l2.rviz',
    ])

    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz', default_value='false', description='Start RViz2',
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config],
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_rviz')),
    )

    return LaunchDescription([
        lidar_ip_arg, local_ip_arg, use_rviz_arg, lidar_node, rviz_node,
    ])
