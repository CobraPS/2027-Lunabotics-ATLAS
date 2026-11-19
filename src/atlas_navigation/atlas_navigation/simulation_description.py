"""Derive the ROS LiDAR transform from the SDF used to spawn the rover."""

import math
import xml.etree.ElementTree as ET


LIDAR_FRAME = 'unilidar_lidar'


def lidar_transform_arguments(model_path):
    """Read the base-relative sensor pose for tf2's static transform publisher."""
    pose = ET.parse(model_path).find("model/link[@name='unilidar_lidar']/pose")
    if pose is None or pose.get('relative_to', 'base_link') not in ('', 'base_link'):
        raise ValueError('LiDAR pose must be expressed relative to base_link')
    values = [float(value) for value in (pose.text or '').split()]
    if len(values) != 6 or not all(math.isfinite(value) for value in values):
        raise ValueError('LiDAR pose must contain six finite XYZ/RPY values')
    arguments = [item for axis, value in zip(('x', 'y', 'z', 'roll', 'pitch', 'yaw'), values)
                 for item in (f'--{axis}', str(value))]
    return arguments + ['--frame-id', 'base_link', '--child-frame-id', LIDAR_FRAME]


def imu_transform_arguments(model_path):
    """Use the SDK-aligned IMU offset declared in the simulated sensor housing."""
    pose = ET.parse(model_path).find("model/link[@name='unilidar_lidar']/sensor[@name='imu']/pose")
    values = [float(value) for value in pose.text.split()]
    return [item for axis, value in zip(('x', 'y', 'z', 'roll', 'pitch', 'yaw'), values)
            for item in (f'--{axis}', str(value))] + [
                '--frame-id', LIDAR_FRAME, '--child-frame-id', 'unilidar_imu']
