"""Guard navigation integration against frame, footprint, and command regressions."""

import ast
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import pytest
import yaml


PACKAGE = Path(__file__).resolve().parents[1]


def parameters(name):
    """Load the installed-format ROS parameter document."""
    return yaml.safe_load((PACKAGE / 'config' / name).read_text())


def test_slam_uses_existing_exact_time_tf_chain():
    """Upstream base_footprint defaults cannot resolve ATLAS odometry."""
    slam = parameters('slam.yaml')['slam_toolbox']['ros__parameters']
    assert slam['base_frame'] == 'base_link'
    assert slam['odom_frame'] == 'odom'
    assert slam['map_frame'] == 'map'
    assert slam['scan_topic'] == '/scan'
    assert slam['use_sim_time'] is True
    assert slam['restamp_tf'] is False
    assert 'map_file_name' not in slam
    # Processing range must stay within the simulated sensor's usable range
    sensor_max_text = ET.parse(PACKAGE / 'models' / 'atlas' / 'model.sdf').findtext(
        ".//sensor[@name='lidar']/lidar/range/max")
    assert sensor_max_text is not None
    sensor_max = float(sensor_max_text)
    assert 0 < slam['min_laser_range'] < slam['max_laser_range'] <= sensor_max


def model_transport_envelope():
    """Actual collision extents of the rover's spawn model (no symmetry assumption)."""
    model = ET.parse(PACKAGE / 'models' / 'atlas' / 'model.sdf').find('model')
    assert model is not None
    min_x = min_y = float('inf')
    max_x = max_y = float('-inf')
    for link in model.findall('link'):
        link_pose_text = link.findtext('pose', '0 0 0 0 0 0')
        assert link_pose_text is not None
        link_pose = [float(v) for v in link_pose_text.split()]
        for shape in link.findall('collision'):
            shape_pose_text = shape.findtext('pose', '0 0 0 0 0 0')
            assert shape_pose_text is not None
            pose = [float(v) for v in shape_pose_text.split()]
            box = shape.find('geometry/box')
            if box is not None:
                box_text = box.findtext('size')
                assert box_text is not None
                sizes = [float(v) for v in box_text.split()]
            else:
                cylinder = shape.find('geometry/cylinder')
                assert cylinder is not None
                radius_text = cylinder.findtext('radius')
                length_text = cylinder.findtext('length')
                assert radius_text is not None and length_text is not None
                radius, length = float(radius_text), float(length_text)
                sizes = [2 * radius, length, 2 * radius]
            # Update independent min/max extents
            min_x = min(min_x, link_pose[0] + pose[0] - sizes[0] / 2)
            max_x = max(max_x, link_pose[0] + pose[0] + sizes[0] / 2)
            min_y = min(min_y, link_pose[1] + pose[1] - sizes[1] / 2)
            max_y = max(max_y, link_pose[1] + pose[1] + sizes[1] / 2)
    assert min_x < max_x and min_y < max_y
    return min_x, max_x, min_y, max_y


def test_costmaps_cover_transport_geometry_and_use_real_scan():
    """Both costmaps must protect the complete footprint, including excavation."""
    config = parameters('navigation.yaml')
    min_x, max_x, min_y, max_y = model_transport_envelope()
    for name in ['local_costmap', 'global_costmap']:
        params = config[name][name]['ros__parameters']
        points = ast.literal_eval(params['footprint'])
        # The declared footprint must cover all four extents of the model's
        # collision geometry, independently of any symmetry assumption.
        assert max(p[0] for p in points) >= max_x
        assert min(p[0] for p in points) <= min_x
        assert max(p[1] for p in points) >= max_y
        assert min(p[1] for p in points) <= min_y
        radius = max(math.hypot(abs(x)+params['footprint_padding'],
                                abs(y)+params['footprint_padding']) for x, y in points)
        assert params['inflation_layer']['inflation_radius'] >= radius
        assert params['robot_base_frame'] == 'base_link'
        assert params['use_sim_time'] is True
        scan = params['obstacle_layer']['scan']
        assert scan['topic'] == '/scan'
        assert scan['marking'] and scan['clearing']
        assert scan['inf_is_valid'] is True
        assert scan['obstacle_min_range'] == pytest.approx(0.05)
        assert scan['raytrace_min_range'] == pytest.approx(0.05)
        assert scan['obstacle_max_range'] < 12.0
        if name == 'global_costmap':
            assert params['track_unknown_space'] is True
            assert params['obstacle_layer']['combination_method'] == 0


def test_navigation_limits_match_drivetrain_and_twist_interface():
    """Controllers must stay below drivetrain limits and retain unstamped Twist."""
    config = parameters('navigation.yaml')
    model = ET.parse(PACKAGE / 'models' / 'atlas' / 'model.sdf')
    drive = model.find("model/plugin[@name='gz::sim::systems::DiffDrive']")
    linear = float(drive.findtext('max_linear_velocity'))
    angular = float(drive.findtext('max_angular_velocity'))
    smoother = config['velocity_smoother']['ros__parameters']
    assert 0 < smoother['max_velocity'][0] <= linear
    assert 0 < smoother['max_velocity'][2] <= angular
    assert smoother['max_velocity'][1] == 0
    assert smoother['velocity_timeout'] < 0.5
    controller = config['controller_server']['ros__parameters']
    assert controller['FollowPath']['desired_linear_vel'] <= smoother['max_velocity'][0]
    for name, params in config.items():
        if name.endswith('costmap'):
            continue
        assert params['ros__parameters']['use_sim_time'] is True
    for name in ['controller_server', 'behavior_server', 'velocity_smoother']:
        assert config[name]['ros__parameters']['enable_stamped_cmd_vel'] is False
    # Goal tolerance must remain tighter than the integration verifier's
    # 0.25 m map-error acceptance threshold, so passing goals in the
    # verifier cannot succeed through a loose configured tolerance alone.
    assert 0 < controller['goal_checker']['xy_goal_tolerance'] < 0.25


def test_lattice_primitives_match_costmaps_and_controller():
    """Use the differential control set at the real costmap resolution, without reversing."""
    import json
    import runpy
    from launch import LaunchContext
    from launch.actions import DeclareLaunchArgument
    from launch.utilities import perform_substitutions

    params = parameters('navigation.yaml')
    planner = params['planner_server']['ros__parameters']
    assert planner['planner_plugins'] == ['GridBased']
    assert planner['GridBased']['plugin'] == 'nav2_smac_planner::SmacPlannerLattice'
    assert not planner['GridBased']['allow_reverse_expansion']
    assert not params['controller_server']['ros__parameters']['FollowPath']['allow_reversing']
    description = runpy.run_path(str(PACKAGE / 'launch/navigation_simulation.launch.py'))
    argument = next(action for action in description['generate_launch_description']().entities
                    if isinstance(action, DeclareLaunchArgument) and
                    action.name == 'lattice_filepath')
    path = perform_substitutions(LaunchContext(), argument.default_value)
    lattice = json.loads(Path(path).read_text())
    metadata = lattice['lattice_metadata']
    assert metadata['motion_model'] == 'diff'
    for name in ['local_costmap', 'global_costmap']:
        assert metadata['grid_resolution'] == params[name][name]['ros__parameters']['resolution']
    assert any(primitive['trajectory_length'] == 0 for primitive in lattice['primitives'])
    assert planner['GridBased']['rotation_penalty'] > planner['GridBased']['non_straight_penalty']
