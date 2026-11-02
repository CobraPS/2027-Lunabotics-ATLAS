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


def test_valid_command_refreshes_timeout():
    """Verify a valid command records when it was received."""
    bridge = SimpleNamespace(
        linear_x=0.0,
        angular_z=0.0,
        last_cmd_time=None,
        publish_drive_command=Mock(),
        get_logger=Mock(),
    )

    msg = SimpleNamespace(
        linear=SimpleNamespace(x=0.5),
        angular=SimpleNamespace(z=-0.2),
    )

    RoboRIOBridge.cmd_vel_callback(bridge, msg)

    assert bridge.last_cmd_time is not None


def test_invalid_command_does_not_refresh_timeout():
    """Verify an invalid command does not postpone the timeout."""
    bridge = SimpleNamespace(
        linear_x=0.0,
        angular_z=0.0,
        last_cmd_time=None,
        publish_drive_command=Mock(),
        get_logger=Mock(),
    )

    msg = SimpleNamespace(
        linear=SimpleNamespace(x=float('nan')),
        angular=SimpleNamespace(z=-0.2),
    )

    RoboRIOBridge.cmd_vel_callback(bridge, msg)

    assert bridge.last_cmd_time is None


def test_command_is_stale_after_timeout():
    """Verify a command older than the timeout is reported as stale."""
    bridge = SimpleNamespace(last_cmd_time=10.0, cmd_timeout=0.5)

    assert not RoboRIOBridge.command_is_stale(bridge, 10.4)
    assert RoboRIOBridge.command_is_stale(bridge, 10.6)


def test_command_is_not_stale_before_first_command():
    """Verify no timeout is reported before any command was received."""
    bridge = SimpleNamespace(last_cmd_time=None, cmd_timeout=0.5)

    assert not RoboRIOBridge.command_is_stale(bridge, 100.0)


def test_timeout_sets_drive_command_to_zero():
    """Verify a stale command is replaced with zero drive values."""
    bridge = SimpleNamespace(
        linear_x=0.5,
        angular_z=-0.2,
        cmd_timeout=0.5,
        drive_timed_out=False,
        command_is_stale=Mock(return_value=True),
        publish_drive_command=Mock(),
        get_logger=Mock(),
    )

    RoboRIOBridge.check_command_timeout(bridge)

    assert bridge.drive_timed_out is True
    assert bridge.linear_x == 0.0
    assert bridge.angular_z == 0.0
    bridge.publish_drive_command.assert_called_once_with(0.0, 0.0)
    bridge.get_logger.return_value.warning.assert_called_once()


def test_timeout_is_reported_only_once():
    """Verify the zero command and warning are not repeated every cycle."""
    bridge = SimpleNamespace(
        linear_x=0.0,
        angular_z=0.0,
        cmd_timeout=0.5,
        drive_timed_out=True,
        command_is_stale=Mock(return_value=True),
        publish_drive_command=Mock(),
        get_logger=Mock(),
    )

    RoboRIOBridge.check_command_timeout(bridge)

    bridge.publish_drive_command.assert_not_called()
    bridge.get_logger.return_value.warning.assert_not_called()


def test_fresh_command_resumes_after_timeout():
    """Verify normal operation resumes once a fresh command arrives."""
    bridge = SimpleNamespace(
        cmd_timeout=0.5,
        drive_timed_out=True,
        command_is_stale=Mock(return_value=False),
        publish_drive_command=Mock(),
        get_logger=Mock(),
    )

    RoboRIOBridge.check_command_timeout(bridge)

    assert bridge.drive_timed_out is False
    bridge.get_logger.return_value.info.assert_called_once()
    bridge.publish_drive_command.assert_not_called()
