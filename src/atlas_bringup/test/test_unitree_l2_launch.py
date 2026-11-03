"""Test Unitree L2 launch configuration without starting hardware or GUI processes."""

from pathlib import Path
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext, LaunchDescription
from launch.actions import ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.utilities import perform_substitutions
from launch_ros.actions import Node
import pytest
import yaml


@pytest.fixture
def launch_description():
    """Load the source launch file so uninstalled launch edits are also checked."""
    launch_file = Path(__file__).resolve().parents[1] / 'launch' / 'unitree_l2.launch.py'
    source = PythonLaunchDescriptionSource(str(launch_file))
    return source.get_launch_description(LaunchContext())


@pytest.fixture
def prepare_launch(launch_description, monkeypatch, tmp_path):
    """Resolve real node commands while replacing only process execution."""
    commands = []

    def capture_command(action, context):
        commands.append((action, [
            perform_substitutions(context, argument) for argument in action.cmd
        ]))
        return None

    # Node.visit still evaluates conditions and prepares real ROS parameter files.
    # The base process action is intercepted before any executable can start.
    monkeypatch.setattr(ExecuteProcess, 'execute', capture_command)
    monkeypatch.setattr(tempfile, 'tempdir', str(tmp_path))
    monkeypatch.delenv('DISPLAY', raising=False)
    monkeypatch.delenv('WAYLAND_DISPLAY', raising=False)

    def prepare(**overrides):
        context = LaunchContext()
        context.launch_configurations.update(overrides)
        for entity in launch_description.entities:
            entity.visit(context)
        return commands

    return prepare


def _command_for(commands, package):
    """Find the single prepared command belonging to a ROS package."""
    matches = [
        command for action, command in commands
        if isinstance(action, Node) and action.node_package == package
    ]
    assert len(matches) == 1, f'Expected one command from {package}, got {matches}'
    return matches[0]


def _parameters(command):
    """Read the effective parameters from the driver's generated YAML files."""
    parameters = {}
    for index, argument in enumerate(command):
        if argument == '--params-file':
            parameter_file = Path(command[index + 1])
            data = yaml.safe_load(parameter_file.read_text())
            for node_parameters in data.values():
                parameters.update(node_parameters['ros__parameters'])
    return parameters


def test_launch_description_loads(launch_description):
    """Reject invalid launch entities as well as import or construction errors."""
    assert isinstance(launch_description, LaunchDescription)
    arguments = launch_description.get_launch_arguments()
    assert {'lidar_ip', 'local_ip', 'use_rviz'} <= {
        argument.name for argument in arguments
    }


def test_unitree_driver_executable(prepare_launch):
    """Configure the installed Unitree ROS 2 driver executable."""
    command = _command_for(prepare_launch(), 'unitree_lidar_ros2')
    assert Path(command[0]).name == 'unitree_lidar_ros2_node'


def test_driver_ethernet_initialization_parameters(prepare_launch):
    """Preserve the fixed Unitree L2 Ethernet initialization settings."""
    command = _command_for(prepare_launch(), 'unitree_lidar_ros2')
    parameters = _parameters(command)
    assert parameters['initialize_type'] == 2
    assert parameters['work_mode'] == 0
    assert parameters['lidar_port'] == 6101
    assert parameters['local_port'] == 6201


@pytest.mark.parametrize(
    'overrides, expected_lidar_ip, expected_local_ip',
    [
        ({}, '10.0.9.3', '10.0.9.1'),
        ({'lidar_ip': '192.0.2.20'}, '192.0.2.20', '10.0.9.1'),
        ({'local_ip': '192.0.2.30'}, '10.0.9.3', '192.0.2.30'),
    ],
    ids=['defaults', 'lidar-ip-override', 'local-ip-override'],
)
def test_driver_ip_parameters(
    prepare_launch, overrides, expected_lidar_ip, expected_local_ip,
):
    """Pass each IP argument through to its corresponding driver parameter."""
    command = _command_for(prepare_launch(**overrides), 'unitree_lidar_ros2')
    parameters = _parameters(command)
    assert parameters['lidar_ip'] == expected_lidar_ip
    assert parameters['local_ip'] == expected_local_ip


def test_rviz_disabled_by_default(prepare_launch):
    """Prepare no RViz process when use_rviz is omitted."""
    commands = prepare_launch()
    assert not any(action.node_package == 'rviz2' for action, _ in commands)


def test_rviz_enabled_when_requested(prepare_launch):
    """Prepare RViz when the use_rviz launch argument is true."""
    command = _command_for(prepare_launch(use_rviz='true'), 'rviz2')
    assert Path(command[0]).name == 'rviz2'


def test_rviz_uses_installed_configuration(prepare_launch):
    """Pass the installed bringup RViz configuration to RViz's -d option."""
    command = _command_for(prepare_launch(use_rviz='true'), 'rviz2')
    assert '-d' in command
    configuration = Path(command[command.index('-d') + 1])
    package_share = Path(get_package_share_directory('atlas_bringup'))
    assert configuration == package_share / 'rviz' / 'unitree_l2.rviz'
    assert configuration.is_file(), f'RViz configuration is not installed: {configuration}'
