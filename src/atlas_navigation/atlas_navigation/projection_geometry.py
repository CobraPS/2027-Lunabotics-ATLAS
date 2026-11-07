"""Static self geometry and ray directions from the SDF actually spawned in Gazebo."""

import math
import xml.etree.ElementTree as ET

import numpy as np


def pose_matrix(element, allowed_frames):
    """Read an explicit XYZ/RPY pose without silently ignoring relative frames."""
    pose = element.find('pose')
    values = [0.0] * 6
    if pose is not None:
        if pose.get('relative_to', '') not in allowed_frames:
            raise ValueError('Unsupported relative frame in projection geometry')
        values = [float(value) for value in pose.text.split()]
    if len(values) != 6 or not np.isfinite(values).all():
        raise ValueError('Projection geometry requires six finite pose values')
    x, y, z, roll, pitch, yaw = values
    cr, sr, cp, sp, cy, sy = (math.cos(roll), math.sin(roll), math.cos(pitch),
                              math.sin(pitch), math.cos(yaw), math.sin(yaw))
    transform = np.eye(4)
    transform[:3, :3] = [[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                         [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                         [-sp, cp * sr, cp * cr]]
    transform[:3, 3] = [x, y, z]
    return transform


class ProjectionGeometry:
    """Use the current rigid transport pose; wheel rotation preserves cylinder shape."""

    def __init__(self, model_path, padding=0.005):
        """Load collision primitives, the sensor extrinsic, and the native ray grid."""
        if not math.isfinite(padding) or padding < 0:
            raise ValueError('Self-filter padding must be finite and nonnegative')
        model = ET.parse(model_path).getroot().find('model')
        base = model.find("link[@name='base_link']")
        base_transform = pose_matrix(base, ('', '__model__'))

        def link_pose(link):
            transform = pose_matrix(link, ('', '__model__', 'base_link'))
            pose = link.find('pose')
            if pose is not None and pose.get('relative_to') == 'base_link':
                return transform
            return np.linalg.inv(base_transform) @ transform

        sensor_link = model.find("link[@name='unilidar_lidar']")
        self.mount = link_pose(sensor_link)
        if not np.allclose(self.mount[:3, 2], [0, 0, 1], atol=1e-8):
            raise ValueError('Planar projection requires an upright LiDAR mount (zero roll/pitch)')
        self.bodies = []
        for link in model.findall('link'):
            link_transform = link_pose(link)
            for collision in link.findall('collision'):
                transform = link_transform @ pose_matrix(collision, ('', link.get('name')))
                box = collision.find('geometry/box')
                cylinder = collision.find('geometry/cylinder')
                if box is not None:
                    size = np.array([float(v) for v in box.findtext('size').split()]) / 2
                    self.bodies.append(('box', transform, size + padding))
                elif cylinder is not None:
                    self.bodies.append(('cylinder', transform, np.array([
                        float(cylinder.findtext('radius')) + padding,
                        float(cylinder.findtext('length')) / 2 + padding])))
                else:
                    raise ValueError('Update self-filter for unsupported SDF collision geometry')
        sensor = sensor_link.find("sensor[@name='lidar']")
        if not np.allclose(pose_matrix(sensor, ('', 'unilidar_lidar')), np.eye(4)):
            raise ValueError('LiDAR rays must originate at the unilidar_lidar frame')
        self.horizontal = int(sensor.findtext('lidar/scan/horizontal/samples'))
        self.vertical = int(sensor.findtext('lidar/scan/vertical/samples'))
        azimuth = np.linspace(float(sensor.findtext('lidar/scan/horizontal/min_angle')),
                              float(sensor.findtext('lidar/scan/horizontal/max_angle')),
                              self.horizontal)
        elevation = np.linspace(float(sensor.findtext('lidar/scan/vertical/min_angle')),
                                float(sensor.findtext('lidar/scan/vertical/max_angle')),
                                self.vertical)
        self.directions = np.stack(np.broadcast_arrays(
            np.cos(elevation[:, None]) * np.cos(azimuth),
            np.cos(elevation[:, None]) * np.sin(azimuth),
            np.sin(elevation[:, None])), axis=-1)
        self.sensor_max = float(sensor.findtext('lidar/range/max'))
        self.azimuth = azimuth

    def to_base(self, points):
        """Transform points from the cloud frame into the ground-centered base_link."""
        return points @ self.mount[:3, :3].T + self.mount[:3, 3]

    def self_mask(self, points):
        """Classify actual 3D body volumes, keeping gaps in the overall footprint."""
        mask = np.zeros(len(points), dtype=bool)
        for kind, transform, size in self.bodies:
            local = (points - transform[:3, 3]) @ transform[:3, :3]
            if kind == 'box':
                mask |= np.all(np.abs(local) <= size, axis=1)
            else:
                mask |= ((np.hypot(local[:, 0], local[:, 1]) <= size[0]) &
                         (np.abs(local[:, 2]) <= size[1]))
        return mask

    def clear_columns(self, points, width, height, min_range, max_range, min_height, max_height):
        """Certify free rays only for the organized native GPU grid, never sparse clouds."""
        if (width, height) != (self.horizontal, self.vertical):
            return np.zeros(self.horizontal, dtype=bool)
        rays = self.directions
        radial = np.hypot(rays[:, :, 0], rays[:, :, 1])
        slope = (rays @ self.mount[:3, :3].T)[:, :, 2] / np.maximum(radial, 1e-9)
        end_range = np.minimum(max_range, self.sensor_max * radial)
        start_z = slope * min_range + self.mount[2, 3]
        end_z = slope * end_range + self.mount[2, 3]
        relevant = ((end_range >= min_range) &
                    (np.minimum(start_z, end_z) <= max_height) &
                    (np.maximum(start_z, end_z) >= min_height))
        full_band = ((end_range >= max_range) & (start_z >= min_height) &
                     (start_z <= max_height) & (end_z >= min_height) & (end_z <= max_height))
        xyz = points.reshape(height, width, 3)
        active = np.abs(rays) > 1e-4
        # +Inf range produces signed Inf XYZ (and possibly NaN on a zero axis).
        # A -Inf near-range failure has opposite signs and must stay unknown.
        no_hit = np.all(~active | (np.isinf(xyz) & (np.sign(xyz) == np.sign(rays))), axis=2)
        farther = (np.isfinite(xyz).all(axis=2) &
                   (np.hypot(xyz[:, :, 0], xyz[:, :, 1]) >= max_range))
        finite = np.isfinite(xyz).all(axis=2)
        # Shape alone does not prove native ray ordering. A shuffled/non-native
        # finite endpoint must not certify free space at a different bearing.
        lengths = np.linalg.norm(xyz[finite], axis=1)
        aligned = np.all(np.abs(xyz[finite] / np.maximum(lengths[:, None], 1e-9) -
                                rays[finite]) < 1e-4, axis=1)
        known = no_hit.copy()
        base = self.to_base(xyz[finite])
        outside_band = (base[:, 2] < min_height) | (base[:, 2] > max_height)
        known[finite] |= (aligned & ~self.self_mask(base) & (outside_band | farther[finite]) &
                          (np.hypot(xyz[finite, 0], xyz[finite, 1]) >= min_range))
        # Never let one clear ray hide an unknown/self-occluded channel at the
        # same azimuth. Some relevant rays leave the height band before 12 m.
        return (np.any(full_band & (no_hit | farther), axis=0) &
                np.all(~relevant | known, axis=0) &
                (min_height <= self.mount[2, 3] <= max_height))
