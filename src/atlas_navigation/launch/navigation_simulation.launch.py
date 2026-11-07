"""Run the ATLAS simulation, online SLAM, and standard Nav2 navigation servers."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, IncludeLaunchDescription
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    """Keep simulation hardware-independent and retain its final command watchdog."""
    share = Path(get_package_share_directory('atlas_navigation'))
    primitives = (Path(get_package_share_directory('nav2_smac_planner')) /
                  'sample_primitives/5cm_resolution/0.5m_turning_radius/diff/output.json')
    slam_share = Path(get_package_share_directory('slam_toolbox'))
    servers = [
        ('nav2_controller', 'controller_server'),
        ('nav2_planner', 'planner_server'),
        ('nav2_behaviors', 'behavior_server'),
        ('nav2_bt_navigator', 'bt_navigator'),
        ('nav2_velocity_smoother', 'velocity_smoother'),
    ]
    input_guard = Node(
        package='atlas_navigation', executable='command_watchdog',
        name='navigation_command_watchdog',
        parameters=[{'cmd_timeout': ParameterValue(
            LaunchConfiguration('nav_cmd_timeout'), value_type=float), 'use_sim_time': True}],
        remappings=[('/cmd_vel', '/navigation/cmd_vel'),
                    ('/simulation/cmd_vel', '/navigation/fresh_cmd_vel')], output='screen')
    actions = [
        DeclareLaunchArgument('cmd_timeout', default_value='0.5'),
        DeclareLaunchArgument('nav_cmd_timeout', default_value='0.3',
                              description='Nav2 input expiry in wall seconds, including pauses'),
        DeclareLaunchArgument('world',
                              default_value=str(share / 'worlds' / 'lunabotics_arena.sdf')),
        DeclareLaunchArgument('lattice_filepath', default_value=str(primitives),
                              description='Differential primitives at costmap resolution'),
        DeclareLaunchArgument('nav_params',
                              default_value=str(share / 'config' / 'navigation.yaml')),
        DeclareLaunchArgument('slam_params', default_value=str(share / 'config' / 'slam.yaml')),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(share / 'launch' / 'simulation.launch.py')),
            launch_arguments={'cmd_timeout': LaunchConfiguration('cmd_timeout'),
                              'world': LaunchConfiguration('world')}.items()),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(slam_share / 'launch' / 'online_async_launch.py')),
            launch_arguments={'use_sim_time': 'true',
                              'slam_params_file': LaunchConfiguration('slam_params')}.items()),
        RegisterEventHandler(OnProcessExit(
            target_action=input_guard,
            on_exit=lambda event, context: [] if context.is_shutdown else [
                EmitEvent(event=Shutdown(reason='Navigation command watchdog exited'))])),
        input_guard,
    ]
    for package, executable in servers:
        remappings = []
        if executable in ('controller_server', 'behavior_server'):
            remappings = [('cmd_vel', '/navigation/cmd_vel')]
        elif executable == 'velocity_smoother':
            remappings = [('cmd_vel', '/navigation/fresh_cmd_vel'),
                          ('cmd_vel_smoothed', '/cmd_vel')]
        actions.append(Node(
            package=package, executable=executable, name=executable,
            parameters=[LaunchConfiguration('nav_params'), {'use_sim_time': True},
                        *([{'GridBased.lattice_filepath': LaunchConfiguration('lattice_filepath')}]
                          if executable == 'planner_server' else [])],
            remappings=remappings, output='screen'))
    actions.append(Node(
        package='nav2_lifecycle_manager', executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        parameters=[{'use_sim_time': True, 'autostart': True,
                     'node_names': [name for _, name in servers], 'bond_timeout': 4.0}],
        output='screen'))
    return LaunchDescription(actions)
