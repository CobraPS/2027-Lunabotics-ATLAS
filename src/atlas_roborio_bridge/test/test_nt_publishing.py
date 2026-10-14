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
