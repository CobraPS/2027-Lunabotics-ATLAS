"""Projection must use observed 3D returns and leave unobserved bearings empty."""

import math

import pytest
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header


def test_height_filtered_minimum_ranges_and_missing_returns(geometry):
    """Reject invalid/self/high points and keep only actual returns per bearing."""
    from atlas_navigation.cloud_to_scan import project_cloud

    header = Header(frame_id='unilidar_lidar')
    header.stamp.sec = 12
    cloud = point_cloud2.create_cloud_xyz32(header, [
        (2.5, 0, 0.1), (2.0, 0, 0.2), (0, 2.0, 0.1),
        (-2.0, 0, 2.0), (0, -1.0, -0.46), (0.4, 0, 0.08),
        (math.nan, 0, 0), (math.inf, 0, 0),
    ])
    scan = project_cloud(cloud, geometry, bins=361)
    assert scan.header == header
    assert scan.ranges[180] == pytest.approx(2.0)
    assert scan.ranges[270] == pytest.approx(2.0)
    assert sum(math.isfinite(value) for value in scan.ranges) == 2
    assert math.isnan(scan.ranges[90])
    assert math.isnan(scan.ranges[360])


def test_empty_cloud_does_not_invent_obstacles(geometry):
    """An empty observation remains empty after projection."""
    from atlas_navigation.cloud_to_scan import project_cloud

    cloud = point_cloud2.create_cloud_xyz32(Header(frame_id='unilidar_lidar'), [])
    scan = project_cloud(cloud, geometry)
    assert all(math.isnan(value) for value in scan.ranges)


@pytest.fixture
def geometry():
    """Use the exact SDF used by the simulated sensor."""
    from pathlib import Path
    from atlas_navigation.projection_geometry import ProjectionGeometry

    return ProjectionGeometry(Path(__file__).resolve().parents[1] /
                              'models' / 'atlas' / 'model.sdf')


def test_close_external_and_low_observed_returns_survive(geometry):
    """The old .8 m circle and sensor-relative .41 m floor lost these returns."""
    from atlas_navigation.cloud_to_scan import project_cloud

    cloud = point_cloud2.create_cloud_xyz32(Header(frame_id='unilidar_lidar'), [
        (0, 0.50, 0.03),       # Outside the chassis, between the wheel axles.
        (0, -3.5, -0.30),     # Observed 16 cm-high point within the -6 degree FOV.
        (0, 2.0, -0.46),      # Level ground must not become an obstacle.
    ])
    scan = project_cloud(cloud, geometry, bins=361)
    assert scan.ranges[270] == pytest.approx(0.50)
    assert scan.ranges[90] == pytest.approx(3.5)
    assert scan.range_min == pytest.approx(0.05)


def test_self_geometry_and_empty_gaps(geometry):
    """Reject actual chassis, wheel, arm and drum surfaces, preserving empty gaps."""
    import numpy as np

    points = np.array([[0, 0.24, 0.4], [0.41, 0.36715, 0.2159],
                       [0.50, 0, 0.54], [0.61, 0.17, 0.68],
                       [0, 0.50, 0.49], [0, 0.35, 0.16]])
    assert geometry.self_mask(points).tolist() == [True, True, True, True, False, False]


def native_cloud(geometry, points):
    """Preserve Gazebo's organized row-major layout for ray-aware clearing tests."""
    import numpy as np

    cloud = point_cloud2.create_cloud_xyz32(
        Header(frame_id='unilidar_lidar'), np.asarray(points).reshape(-1, 3))
    cloud.width, cloud.height = geometry.horizontal, geometry.vertical
    cloud.row_step = cloud.width * cloud.point_step
    return cloud


def test_only_observed_eligible_rays_certify_clearing(geometry):
    """Sky, invalid, near-range failure and self-only directions stay unknown."""
    import numpy as np
    from atlas_navigation.cloud_to_scan import project_cloud

    rays = geometry.directions
    with np.errstate(invalid='ignore'):
        raw = rays * np.inf
    raw[:, 0] = np.nan
    raw[:, 200] = np.nan
    raw[:, 300] = np.nan
    row = 1  # -1.2 degrees, stays above the .05 m floor through 12 m.
    with np.errstate(invalid='ignore'):
        raw[row, 100] = rays[row, 100] * np.inf
        raw[-1, 200] = rays[-1, 200] * np.inf  # Sky cannot clear the navigation band.
        raw[row, 300] = rays[row, 300] * -np.inf
    raw[row, 400] = rays[row, 400] * 20  # Actual farther endpoint also certifies free space.
    raw[3, 100] = rays[3, 100] * 3      # A real obstacle wins over a higher free ray.
    scan = project_cloud(native_cloud(geometry, raw), geometry)
    assert math.isfinite(scan.ranges[100])
    assert math.isinf(scan.ranges[400])
    assert math.isnan(scan.ranges[200])
    assert math.isnan(scan.ranges[300])
    assert math.isnan(scan.ranges[0])
    with np.errstate(invalid='ignore'):
        raw[3, 100] = rays[3, 100] * np.inf
    scan = project_cloud(native_cloud(geometry, raw), geometry)
    assert scan.ranges[100] == math.inf


def test_unobserved_near_body_directions_do_not_clear(geometry):
    """Removing a self return does not establish free space behind that body."""
    import numpy as np
    from atlas_navigation.cloud_to_scan import project_cloud

    raw = np.full(geometry.directions.shape, np.nan)
    # Return at the top of a front arm (base Z .54), in an otherwise unknown column.
    raw[4, 281] = [0.4, 0, 0.08]
    scan = project_cloud(native_cloud(geometry, raw), geometry)
    assert math.isnan(scan.ranges[281])


def test_mount_transform_and_wrong_frame(tmp_path):
    """A translated and yaw-rotated provisional mount must still reject body returns."""
    from pathlib import Path
    import numpy as np
    import xacro
    from atlas_navigation.cloud_to_scan import project_cloud
    from atlas_navigation.projection_geometry import ProjectionGeometry

    model = Path(__file__).resolve().parents[1] / 'models/atlas/model.sdf.xacro'
    document = xacro.process_file(str(model), mappings={
        'lidar_x': '0.1', 'lidar_y': '-0.03', 'lidar_z': '0.53',
        'lidar_yaw': '0.5'})
    path = tmp_path / 'model.sdf'
    path.write_text(document.toxml())
    geometry = ProjectionGeometry(path)
    base = np.array([[0, .5, .49], [.5, 0, .54]])
    local = (base - geometry.mount[:3, 3]) @ geometry.mount[:3, :3]
    cloud = point_cloud2.create_cloud_xyz32(Header(frame_id='unilidar_lidar'), local)
    scan = project_cloud(cloud, geometry)
    assert sum(math.isfinite(r) for r in scan.ranges) == 1
    cloud.header.frame_id = 'base_link'
    with pytest.raises(ValueError, match='unilidar_lidar'):
        project_cloud(cloud, geometry)


@pytest.mark.parametrize('axis', ['roll', 'pitch'])
def test_tilted_mount_is_rejected(tmp_path, axis):
    """A tilted cloud needs a separate horizontal scan frame, never a false XY projection."""
    from pathlib import Path
    import xacro
    from atlas_navigation.projection_geometry import ProjectionGeometry

    model = Path(__file__).resolve().parents[1] / 'models/atlas/model.sdf.xacro'
    document = xacro.process_file(str(model), mappings={f'lidar_{axis}': '0.1'})
    path = tmp_path / 'model.sdf'
    path.write_text(document.toxml())
    with pytest.raises(ValueError, match='upright'):
        ProjectionGeometry(path)


def test_unknown_channel_blocks_other_clear_rays(geometry):
    """Do not erase an obstacle merely because another height channel is clear."""
    import numpy as np
    from atlas_navigation.cloud_to_scan import project_cloud

    with np.errstate(invalid='ignore'):
        raw = geometry.directions * np.inf
    raw[2, 100] = np.nan
    raw[4, 200] = [0.4, 0, 0.08]  # A self-occluded relevant channel.
    scan = project_cloud(native_cloud(geometry, raw), geometry)
    assert math.isnan(scan.ranges[100])
    assert math.isnan(scan.ranges[200])
    assert math.isinf(scan.ranges[400])


def test_native_dimensions_alone_cannot_certify_free_space(geometry):
    """Repeated forward returns must never clear unrelated native-grid bearings."""
    import numpy as np
    from atlas_navigation.cloud_to_scan import project_cloud

    raw = np.full(geometry.directions.shape, [20.0, 0.0, 0.0])
    scan = project_cloud(native_cloud(geometry, raw), geometry)
    assert all(math.isnan(value) for value in scan.ranges)


def test_coarse_bins_preserve_unknown_native_columns(geometry):
    """One free native ray must not hide its unknown neighbor in a coarser bin."""
    import numpy as np
    from atlas_navigation.cloud_to_scan import project_cloud

    with np.errstate(invalid='ignore'):
        raw = geometry.directions * np.inf
    raw[:, 1] = np.nan
    scan = project_cloud(native_cloud(geometry, raw), geometry, bins=361)
    assert math.isnan(scan.ranges[1])
    assert math.isinf(scan.ranges[100])


def test_padded_rows_preserve_obstacle_coordinates(geometry):
    """PointCloud2 row padding is not a point and must not hide the next return."""
    from array import array
    import struct
    from atlas_navigation.cloud_to_scan import project_cloud

    cloud = point_cloud2.create_cloud_xyz32(
        Header(frame_id='unilidar_lidar'), [(2.0, 0.0, .1), (0.0, 3.0, .1)])
    cloud.width, cloud.height = 1, 2
    cloud.row_step = 24
    padding = struct.pack('<fff', math.nan, math.nan, math.nan)
    cloud.data = array('B', bytes(cloud.data[:12]) + padding + bytes(cloud.data[12:]) + padding)
    scan = project_cloud(cloud, geometry, bins=361)
    assert scan.ranges[180] == pytest.approx(2.0)
    assert scan.ranges[270] == pytest.approx(3.0)


@pytest.mark.parametrize('damage', ['missing_z', 'vector_z', 'short_data', 'short_row'])
def test_malformed_cloud_is_rejected_as_input_error(geometry, damage):
    """Invalid schemas/layouts must not escape the callback's ValueError handling."""
    from atlas_navigation.cloud_to_scan import project_cloud

    cloud = point_cloud2.create_cloud_xyz32(Header(frame_id='unilidar_lidar'), [(2, 0, .1)])
    if damage == 'missing_z':
        cloud.fields = cloud.fields[:2]
    elif damage == 'vector_z':
        cloud.fields[2].count = 2
    elif damage == 'short_data':
        cloud.data = cloud.data[:4]
    else:
        cloud.row_step = 4
    with pytest.raises(ValueError):
        project_cloud(cloud, geometry)


def test_projector_recovers_after_malformed_message():
    """A rejected message cannot prevent the next valid scan from being published."""
    import time
    import uuid
    from atlas_navigation.cloud_to_scan import CloudToScan
    import rclpy
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import LaserScan

    topic = '/projection_test_' + uuid.uuid4().hex
    rclpy.init(args=['--ros-args', '-r', '/scan:=' + topic,
                     '-r', '/unilidar/cloud:=' + topic + '/cloud',
                     '-r', '__node:=projection_recovery_test'])
    node = CloudToScan()
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    received = []
    subscription = node.create_subscription(
        LaserScan, topic, received.append, qos_profile_sensor_data)
    valid = point_cloud2.create_cloud_xyz32(Header(frame_id='unilidar_lidar'), [(2, 0, .1)])
    malformed = point_cloud2.create_cloud_xyz32(Header(frame_id='unilidar_lidar'), [(2, 0, .1)])
    malformed.fields = malformed.fields[:2]
    try:
        node._on_cloud(malformed)
        deadline = time.monotonic() + 3
        while not received:
            assert time.monotonic() < deadline, 'Valid scan did not follow malformed input'
            node._on_cloud(valid)
            executor.spin_once(timeout_sec=.02)
        assert received[-1].ranges[281] == pytest.approx(2.0)
    finally:
        node.destroy_subscription(subscription)
        executor.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()


@pytest.mark.parametrize('bigendian', [False, True])
@pytest.mark.parametrize('code, datatype, size', [('f', 7, 4), ('d', 8, 8)])
def test_xyz_reader_respects_field_offsets_and_byte_order(
        geometry, bigendian, code, datatype, size):
    """Reordered float fields in either byte order preserve ranges and input bytes."""
    from array import array
    import struct
    from atlas_navigation.cloud_to_scan import project_cloud
    from sensor_msgs.msg import PointCloud2, PointField

    data = struct.pack(('>' if bigendian else '<') + code * 3, .1, 0.0, 2.5)
    cloud = PointCloud2(
        header=Header(frame_id='unilidar_lidar'), width=1, height=1,
        fields=[PointField(name=name, offset=index * size, datatype=datatype, count=1)
                for index, name in enumerate(['z', 'y', 'x'])],
        is_bigendian=bigendian, point_step=3 * size, row_step=3 * size, data=array('B', data))
    scan = project_cloud(cloud, geometry, bins=361)
    assert scan.ranges[180] == pytest.approx(2.5)
    assert bytes(cloud.data) == data
