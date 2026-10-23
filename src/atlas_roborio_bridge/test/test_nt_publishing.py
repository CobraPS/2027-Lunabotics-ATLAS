"""Tests for NetworkTables drive command publishing."""

from types import SimpleNamespace
from unittest.mock import Mock

from atlas_roborio_bridge.roborio_bridge import RoboRIOBridge


def test_publish_drive_command():
    """Verify both drive command values are published correctly."""
    forward_pub = Mock()
    turn_pub = Mock()

    bridge = SimpleNamespace(
        drive_forward_pub=forward_pub,
        drive_turn_pub=turn_pub,
    )

    RoboRIOBridge.publish_drive_command(bridge, 0.5, -0.2)
    forward_pub.set.assert_called_once_with(0.5)
    turn_pub.set.assert_called_once_with(-0.2)


def test_cmd_vel_callback_publishes_valid_command():
    """Verify valid ROS 2 drive commands are forwarded to NetworkTables."""
    bridge = SimpleNamespace(
        linear_x=0.0,
        angular_z=0.0,
        publish_drive_command=Mock(),
        get_logger=Mock(),
    )

    msg = SimpleNamespace(
        linear=SimpleNamespace(x=0.5),
        angular=SimpleNamespace(z=-0.2),
    )

    RoboRIOBridge.cmd_vel_callback(bridge, msg)

    assert bridge.linear_x == 0.5
    assert bridge.angular_z == -0.2
    bridge.publish_drive_command.assert_called_once_with(0.5, -0.2)


def test_cmd_vel_callback_rejects_invalid_command():
    """Verify invalid ROS 2 commands are not published."""
    bridge = SimpleNamespace(
        linear_x=0.0,
        angular_z=0.0,
        publish_drive_command=Mock(),
        get_logger=Mock(),
    )

    msg = SimpleNamespace(
        linear=SimpleNamespace(x=float('nan')),
        angular=SimpleNamespace(z=-0.2),
    )

    RoboRIOBridge.cmd_vel_callback(bridge, msg)

    bridge.publish_drive_command.assert_not_called()
    bridge.get_logger.return_value.warning.assert_called_once()
    assert bridge.linear_x == 0.0
    assert bridge.angular_z == 0.0
