"""Project observed L2 returns, distinguishing self/unknown rays from clear rays."""

import math
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from atlas_navigation.projection_geometry import ProjectionGeometry
import numpy as np
from rcl_interfaces.msg import ParameterDescriptor
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan, PointCloud2, PointField
from sensor_msgs_py import point_cloud2


def cloud_xyz(cloud):
    """Read scalar XYZ fields with declared row strides, without mutating the cloud."""
    fields = [field for field in cloud.fields if field.name in ('x', 'y', 'z')]
    if (len(fields) != 3 or {field.name for field in fields} != {'x', 'y', 'z'} or
            any(field.count != 1 or field.datatype not in (PointField.FLOAT32, PointField.FLOAT64)
                for field in fields)):
        raise ValueError('PointCloud2 requires scalar floating-point x, y and z fields')
    if (cloud.point_step <= 0 or cloud.row_step < cloud.width * cloud.point_step or
            len(cloud.data) != cloud.row_step * cloud.height):
        raise ValueError('Invalid PointCloud2 strides or data length')
    dtype = point_cloud2.dtype_from_fields(fields, point_step=cloud.point_step).newbyteorder(
        '>' if cloud.is_bigendian else '<')
    points = np.ndarray((cloud.height, cloud.width), dtype=dtype, buffer=cloud.data,
                        strides=(cloud.row_step, cloud.point_step))
    return np.stack([points[axis] for axis in ('x', 'y', 'z')], axis=-1).reshape(-1, 3)


def project_cloud(cloud, geometry, bins=563, min_height=0.05, max_height=1.5,
                  min_range=0.05, max_range=12.0, scan_time=1 / 5.55):
    """Take minimum XY ranges from non-self points in a base_link height band."""
    if cloud.header.frame_id != 'unilidar_lidar':
        raise ValueError('Expected PointCloud2 in unilidar_lidar')
    scan = LaserScan()
    scan.header = cloud.header
    scan.angle_min = -math.pi
    scan.angle_max = math.pi
    scan.angle_increment = 2 * math.pi / (bins - 1)
    scan.range_min, scan.range_max, scan.scan_time = min_range, max_range, scan_time
    ranges = np.full(bins, np.inf, dtype=np.float32)
    known_clear = np.zeros(bins, dtype=bool)
    if cloud.width * cloud.height:
        raw = cloud_xyz(cloud)
        clear = geometry.clear_columns(raw, cloud.width, cloud.height,
                                       min_range, max_range, min_height, max_height)
        columns = np.rint((geometry.azimuth + math.pi) / scan.angle_increment).astype(int)
        columns = columns.clip(0, bins - 1)
        known_clear = np.bincount(columns, minlength=bins) > 0
        # Every native column contributing to a coarser output bin must be clear.
        np.logical_and.at(known_clear, columns, clear)
        points = raw[np.isfinite(raw).all(axis=1)]
        base = geometry.to_base(points)
        keep = ((base[:, 2] >= min_height) & (base[:, 2] <= max_height) &
                ~geometry.self_mask(base))
        points = points[keep]
        radial = np.hypot(points[:, 0], points[:, 1])
        valid = (radial >= min_range) & (radial < max_range)
        points, radial = points[valid], radial[valid]
        indices = np.rint((np.arctan2(points[:, 1], points[:, 0]) + math.pi) /
                          scan.angle_increment).astype(int).clip(0, bins - 1)
        np.minimum.at(ranges, indices, radial)
    ranges[np.isinf(ranges) & ~known_clear] = np.nan
    scan.ranges = ranges.tolist()
    return scan


class CloudToScan(Node):
    """Publish the navigation scan without treating missing observations as free."""

    def __init__(self):
        """Load the same model as Gazebo and configure ground-relative filtering."""
        super().__init__('l2_cloud_to_scan')
        defaults = {'bins': 563, 'min_height': 0.05, 'max_height': 1.5,
                    'min_range': 0.05, 'max_range': 12.0, 'scan_time': 1 / 5.55}
        self._settings = {
            key: self.declare_parameter(key, value, ParameterDescriptor(read_only=True)).value
            for key, value in defaults.items()
        }
        p = self._settings
        if (p['bins'] < 2 or not all(math.isfinite(value) for value in p.values()) or
                p['max_height'] < p['min_height'] or not 0 < p['min_range'] < p['max_range'] or
                p['scan_time'] <= 0):
            raise ValueError('Invalid cloud-to-scan projection settings')
        model_default = (Path(get_package_share_directory('atlas_navigation')) /
                         'models/atlas/model.sdf')
        descriptor = ParameterDescriptor(read_only=True)
        model = self.declare_parameter('model_path', str(model_default), descriptor).value
        padding = self.declare_parameter('self_padding', 0.005, descriptor).value
        self._geometry = ProjectionGeometry(model, padding)
        self._publisher = self.create_publisher(LaserScan, '/scan', qos_profile_sensor_data)
        self._subscription = self.create_subscription(
            PointCloud2, '/unilidar/cloud', self._on_cloud, qos_profile_sensor_data)

    def _on_cloud(self, cloud):
        try:
            self._publisher.publish(project_cloud(cloud, self._geometry, **self._settings))
        except ValueError as error:
            self.get_logger().error(str(error))


def main(args=None):
    """Run the projection independently of physical sensor drivers."""
    rclpy.init(args=args)
    node = None
    try:
        node = CloudToScan()
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()
