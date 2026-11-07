"""Keep the ROS sensor transform aligned with the configured Gazebo mount."""

from pathlib import Path
import xml.etree.ElementTree as ET

import pytest
import xacro


MODEL_DIR = Path(__file__).resolve().parents[1] / 'models' / 'atlas'


@pytest.mark.parametrize('mount, expected', [
    ({}, ['0.0', '0.0', '0.46', '0.0', '0.0', '0.0']),
    ({'lidar_x': '0.12', 'lidar_y': '-0.03', 'lidar_z': '0.53',
      'lidar_roll': '0.01', 'lidar_pitch': '0.05', 'lidar_yaw': '0.2'},
     ['0.12', '-0.03', '0.53', '0.01', '0.05', '0.2']),
])
def test_tf_arguments_follow_sensor_mount(tmp_path, mount, expected):
    """A mounting change must affect both the sensor and its ROS transform."""
    document = xacro.process_file(str(MODEL_DIR / 'model.sdf.xacro'), mappings=mount)
    model = ET.fromstring(document.toxml())
    sensor = model.find("model/link[@name='unilidar_lidar']/sensor[@name='lidar']")
    assert sensor is not None, 'The rover must contain the simulated LiDAR'
    assert sensor.get('type') == 'gpu_lidar'
    model_path = tmp_path / 'model.sdf'
    model_path.write_text(document.toxml())

    from atlas_navigation.simulation_description import lidar_transform_arguments

    arguments = lidar_transform_arguments(model_path)
    values = dict(zip(arguments[::2], arguments[1::2]))
    assert [values[f'--{axis}'] for axis in ('x', 'y', 'z', 'roll', 'pitch', 'yaw')] == expected
    assert values['--frame-id'] == 'base_link'
    assert values['--child-frame-id'] == 'unilidar_lidar'


@pytest.mark.parametrize('pose, relative_to', [
    ('0 0 nan 0 0 0', ''),
    ('0 0 0.7', ''),
    ('0 0 0.7 0 0 0', 'another_frame'),
])
def test_reject_ambiguous_or_invalid_mount(tmp_path, pose, relative_to):
    """Invalid mounts must fail before publishing an incorrect static transform."""
    from atlas_navigation.simulation_description import lidar_transform_arguments

    root = ET.parse(MODEL_DIR / 'model.sdf')
    sensor_pose = root.find("model/link[@name='unilidar_lidar']/pose")
    assert sensor_pose is not None
    sensor_pose.text = pose
    if relative_to:
        sensor_pose.set('relative_to', relative_to)
    path = tmp_path / 'model.sdf'
    root.write(path)
    with pytest.raises(ValueError, match='LiDAR pose'):
        lidar_transform_arguments(path)


@pytest.mark.parametrize('mode, minimum', [('96', -6), ('90', 0)])
def test_l2_asymmetric_coverage_and_housing(mode, minimum):
    """Preserve upright coverage, default-mount consistency, and sensor sanity."""
    import math

    document = xacro.process_file(str(MODEL_DIR / 'model.sdf.xacro'), mappings={
        'lidar_vertical_fov': mode})
    model = ET.fromstring(document.toxml()).find('model')
    housing = model.find("link[@name='unilidar_lidar']")
    assert housing is not None, 'L2 needs its own housing and bottom-origin frame'
    # The default mount must produce the same pose as explicitly passing the
    # xacro's own defaults, so editing either path cannot silently diverge.
    pose_text = housing.findtext('pose')
    assert pose_text is not None
    default_pose = [float(v) for v in pose_text.split()]
    explicit = xacro.process_file(str(MODEL_DIR / 'model.sdf.xacro'), mappings={
        'lidar_vertical_fov': mode,
        'lidar_x': str(default_pose[0]), 'lidar_y': str(default_pose[1]),
        'lidar_z': str(default_pose[2]), 'lidar_roll': str(default_pose[3]),
        'lidar_pitch': str(default_pose[4]), 'lidar_yaw': str(default_pose[5])})
    explicit_housing = ET.fromstring(explicit.toxml()).find(
        "model/link[@name='unilidar_lidar']")
    assert explicit_housing is not None
    explicit_text = explicit_housing.findtext('pose')
    assert explicit_text is not None
    assert [float(v) for v in explicit_text.split()] == pytest.approx(default_pose)
    # Upright mount: zero roll and pitch, so the projection stays XY-planar.
    assert default_pose[3:5] == pytest.approx([0, 0])
    # Housing has positive mass and reasonable collision geometry
    mass_text = housing.findtext('inertial/mass')
    assert mass_text is not None and float(mass_text) > 0
    collision_box = housing.find('collision/geometry/box')
    assert collision_box is not None
    size_text = collision_box.findtext('size')
    assert size_text is not None
    size = [float(v) for v in size_text.split()]
    assert all(s > 0 for s in size)
    # Collision pose centers the box on its bottom face
    collision_pose_text = housing.findtext('collision/pose')
    assert collision_pose_text is not None
    assert float(collision_pose_text.split()[2]) == pytest.approx(size[2] / 2)
    sensor = housing.find("sensor[@name='lidar']")
    assert sensor is not None
    # Sensor publishes at a positive rate
    rate_text = sensor.findtext('update_rate')
    assert rate_text is not None and float(rate_text) > 0
    vertical = sensor.find('lidar/scan/vertical')
    assert vertical is not None
    min_text = vertical.findtext('min_angle')
    max_text = vertical.findtext('max_angle')
    samples_text = vertical.findtext('samples')
    assert min_text is not None and max_text is not None and samples_text is not None
    assert float(min_text) == pytest.approx(math.radians(minimum))
    assert float(max_text) == pytest.approx(math.pi / 2)
    assert int(samples_text) > 1
    # Range parameters are positive and ordered
    rmin_text = sensor.findtext('lidar/range/min')
    rmax_text = sensor.findtext('lidar/range/max')
    res_text = sensor.findtext('lidar/range/resolution')
    assert rmin_text is not None and rmax_text is not None and res_text is not None
    rmin, rmax, res = float(rmin_text), float(rmax_text), float(res_text)
    assert 0 < rmin < rmax
    assert res > 0
    # IMU publishes at a positive rate
    imu_rate_text = housing.findtext("sensor[@name='imu']/update_rate")
    assert imu_rate_text is not None and float(imu_rate_text) > 0
