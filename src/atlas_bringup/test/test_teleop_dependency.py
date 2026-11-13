"""Tests for the keyboard teleoperation dependency."""

from ament_index_python.packages import get_package_prefix


def test_teleop_twist_keyboard_is_installed():
    """Verify teleop_twist_keyboard is available in the environment."""
    assert get_package_prefix('teleop_twist_keyboard')
