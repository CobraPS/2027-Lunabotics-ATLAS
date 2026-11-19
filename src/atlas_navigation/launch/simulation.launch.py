"""Spawn the provisional ATLAS rover and ROS interfaces in headless Harmonic."""

import os
from pathlib import Path
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
from atlas_navigation.simulation_description import imu_transform_arguments
from atlas_navigation.simulation_description import LIDAR_FRAME, lidar_transform_arguments
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ros_gz_sim.actions.gzserver import GazeboRosPaths


def generate_launch_description():
    """Start Gazebo, spawn automatically, and bridge simulation topics."""
    package_share = Path(get_package_share_directory('atlas_navigation'))
    model_path = package_share / 'models' / 'atlas' / 'model.sdf'
    world_path = LaunchConfiguration('world')
    model_paths, plugin_paths = GazeboRosPaths.get_paths()
    lidar = ET.parse(model_path).find("model/link[@name='unilidar_lidar']/sensor[@name='lidar']")
    watchdog = Node(
        package='atlas_navigation', executable='command_watchdog',
        parameters=[{'cmd_timeout': ParameterValue(
            LaunchConfiguration('cmd_timeout'), value_type=float)}],
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument('world',
                              default_value=str(package_share / 'worlds' / 'lunabotics_arena.sdf'),
                              description='SDF world with name lunabotics_arena'),
        DeclareLaunchArgument('cmd_timeout', default_value='0.5',
                              description='Positive finite command timeout in wall seconds'),
        DeclareLaunchArgument('scan_min_height', default_value='0.05',
                              description='Projection lower Z bound in base_link (m)'),
        DeclareLaunchArgument('scan_max_height', default_value='1.5',
                              description='Projection upper Z bound in base_link (m)'),
        DeclareLaunchArgument('scan_min_range', default_value='0.05',
                              description='Projection minimum XY range (m)'),
        RegisterEventHandler(OnProcessExit(
            target_action=watchdog,
            on_exit=lambda event, context: [] if context.is_shutdown else [
                EmitEvent(event=Shutdown(reason='Simulation command watchdog exited'))],
        )),
        watchdog,
        # Manage the simulator itself: the upstream shell wrapper can orphan it
        # when ROS launch signals only the intermediate shell during shutdown.
        ExecuteProcess(
            cmd=['gz', 'sim', '-r', '-s', '--headless-rendering', world_path,
                 '--force-version', '8'], output='screen',
            additional_env={
                'GZ_SIM_RESOURCE_PATH': os.pathsep.join([
                    os.environ.get('GZ_SIM_RESOURCE_PATH', ''), model_paths]),
                'GZ_SIM_SYSTEM_PLUGIN_PATH': os.pathsep.join([
                    os.environ.get('GZ_SIM_SYSTEM_PLUGIN_PATH', ''),
                    os.environ.get('LD_LIBRARY_PATH', ''), plugin_paths]),
            },
            on_exit=lambda event, context: [] if context.is_shutdown else [
                EmitEvent(event=Shutdown(reason='Gazebo exited'))]),
        # create waits for the world's spawn service; no startup sleep is needed.
        Node(
            package='ros_gz_sim',
            executable='create',
            arguments=[
                '-world', 'lunabotics_arena',
                '-file', str(model_path),
                '-name', 'atlas', '-allow_renaming', 'false',
                '-x', '1.0', '-y', '1.0', '-z', '0.01',
            ],
            output='screen',
        ),
        Node(
            package='tf2_ros', executable='static_transform_publisher',
            name='atlas_lidar_transform', arguments=lidar_transform_arguments(model_path),
            parameters=[{'use_sim_time': True}], output='screen',
        ),
        # Sensor-only frame overrides preserve the drivetrain's odometry/TF headers.
        Node(
            package='ros_gz_bridge', executable='parameter_bridge', name='atlas_lidar_bridge',
            arguments=[
                '/model/atlas/lidar/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked'],
            remappings=[('/model/atlas/lidar/points', '/unilidar/cloud')],
            parameters=[{'override_frame_id': LIDAR_FRAME, 'use_sim_time': True}],
            output='screen',
        ),
        Node(
            package='atlas_navigation', executable='cloud_to_scan',
            parameters=[{
                'use_sim_time': True,
                'bins': int(lidar.findtext('lidar/scan/horizontal/samples')),
                'max_range': 12.0,
                'model_path': str(model_path),
                'scan_time': 1 / float(lidar.findtext('update_rate')),
                'min_height': ParameterValue(
                    LaunchConfiguration('scan_min_height'), value_type=float),
                'max_height': ParameterValue(
                    LaunchConfiguration('scan_max_height'), value_type=float),
                'min_range': ParameterValue(
                    LaunchConfiguration('scan_min_range'), value_type=float),
            }], output='screen',
        ),
        Node(
            package='tf2_ros', executable='static_transform_publisher',
            name='atlas_imu_transform', arguments=imu_transform_arguments(model_path),
            parameters=[{'use_sim_time': True}], output='screen',
        ),
        Node(
            package='ros_gz_bridge', executable='parameter_bridge', name='atlas_imu_bridge',
            arguments=['/model/atlas/imu@sensor_msgs/msg/Imu[gz.msgs.IMU'],
            remappings=[('/model/atlas/imu', '/unilidar/imu')],
            parameters=[{'override_frame_id': 'unilidar_imu', 'use_sim_time': True}],
            output='screen',
        ),
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='atlas_simulation_bridge',
            parameters=[{
                'config_file': str(package_share / 'config' / 'simulation_bridge.yaml'),
                'use_sim_time': True,
            }],
            output='screen',
        ),
    ])
