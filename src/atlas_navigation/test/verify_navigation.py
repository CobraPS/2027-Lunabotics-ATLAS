"""
Verify repeated State Lattice goals and command stopping in a fresh Lunabotics arena.

Run against navigation_simulation.launch.py in an isolated ROS domain and Gazebo
partition, with no hardware bridge or manual publishers. Gazebo pose is a test
oracle only; navigation receives no ground-truth map or terrain hazard locations.
"""

import argparse
from collections import deque
import json
import math
from pathlib import Path
import subprocess
import threading
import time

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, Twist
from google.protobuf.json_format import Parse
from gz.msgs10.pose_v_pb2 import Pose_V
from lifecycle_msgs.srv import GetState
from nav2_msgs.action import ComputePathToPose, NavigateToPose
from nav2_msgs.srv import ManageLifecycleNodes
from nav_msgs.msg import OccupancyGrid, Odometry
from rcl_interfaces.srv import GetParameters
import rclpy
from rclpy.action import ActionClient
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, qos_profile_sensor_data, QoSProfile
from rclpy.time import Time
from sensor_msgs.msg import Imu, LaserScan, PointCloud2
from tf2_ros import Buffer, TransformListener


SERVERS = ['slam_toolbox', 'controller_server', 'planner_server', 'behavior_server',
           'bt_navigator', 'velocity_smoother']
WORLD = '/world/lunabotics_arena/'
# Return inside the start area, away from the west/south boundary. Online SLAM
# corrections can put the exact initial origin inside the padded wall cost.
GOALS = [(4.0, 2.6, 0.0), (1.0, 2.6, math.pi), (.35, .35, 0.0)]


def yaw(q):
    """Read heading from a quaternion."""
    return math.atan2(2 * (q.w*q.z + q.x*q.y), 1 - 2 * (q.y*q.y + q.z*q.z))


def angle_delta(a, b):
    """Read the shortest signed angular difference."""
    return math.atan2(math.sin(a-b), math.cos(a-b))


def clearance(pose):
    """Check the full transport rectangle against the arena boundary and central column."""
    x, y, heading = pose
    c, s = math.cos(heading), math.sin(heading)
    corners = [(x+c*dx-s*dy, y+s*dx+c*dy)
               for dx, dy in [(.73, .36715), (.73, -.36715), (-.73, -.36715), (-.73, .36715)]]
    assert all(0 < px < 6.8 and 0 < py < 5 for px, py in corners), 'Footprint crossed a duct'
    column = [(3.15, 2.25), (3.65, 2.25), (3.65, 2.75), (3.15, 2.75)]
    gaps = []
    for ax, ay in [(1, 0), (0, 1), (c, s), (-s, c)]:
        rover = [px*ax+py*ay for px, py in corners]
        obstacle = [px*ax+py*ay for px, py in column]
        gaps.append(max(min(rover)-max(obstacle), min(obstacle)-max(rover)))
    return max(gaps)


def occupied_near(grid, x, y):
    """Require occupied cells near the observed column's near face, not just free space."""
    col = int((x-grid.info.origin.position.x)/grid.info.resolution)
    row = int((y-grid.info.origin.position.y)/grid.info.resolution)
    return any(grid.data[r*grid.info.width+c] >= 65
               for r in range(max(0, row-5), min(grid.info.height, row+6))
               for c in range(max(0, col-5), min(grid.info.width, col+6)))


def main():
    """Check sensing, mapping, physical goal arrival, avoidance, cancellation and expiry."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', required=True, help='JSON report; retained on failures too')
    parser.add_argument('--timeout', type=float, default=180.0, help='Wall seconds per goal')
    parser.add_argument('--cmd-timeout', type=float, default=.5)
    parser.add_argument('--nav-cmd-timeout', type=float, default=.3)
    args = parser.parse_args()
    assert all(math.isfinite(v) and v > 0
               for v in [args.timeout, args.cmd_timeout, args.nav_cmd_timeout])
    report = {'passed': False, 'goals': []}
    output = Path(args.output)
    output.write_text(json.dumps(report))  # Fail before motion if the output is unwritable.
    rclpy.init()
    node = rclpy.create_node('atlas_navigation_verifier', parameter_overrides=[
        Parameter('use_sim_time', value=True)])
    state, truth, errors = {}, {}, []
    commands, guarded = deque(maxlen=400), deque(maxlen=400)
    buffer = Buffer(node=node)
    listener = TransformListener(buffer, node)
    process = subprocess.Popen(['gz', 'topic', '--echo', '--json-output',
                                '--topic', WORLD+'pose/info'],
                               stdout=subprocess.PIPE, text=True, start_new_session=True)

    def read_truth():
        try:
            for line in process.stdout:
                for pose in Parse(line, Pose_V()).pose:
                    if pose.name == 'atlas':
                        truth['pose'] = pose
        except Exception as error:
            errors.append(str(error))

    reader = threading.Thread(target=read_truth)
    reader.start()
    transient = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    for kind, topic, key, qos in [
            (Odometry, '/odom', 'odom', 10), (LaserScan, '/scan', 'scan', qos_profile_sensor_data),
            (PointCloud2, '/unilidar/cloud', 'cloud', qos_profile_sensor_data),
            (Imu, '/unilidar/imu', 'imu', qos_profile_sensor_data),
            *[(OccupancyGrid, topic, key, transient) for topic, key in [
                ('/map', 'map'), ('/local_costmap/costmap', 'local'),
                ('/global_costmap/costmap', 'global')]]]:
        node.create_subscription(kind, topic, lambda m, key=key: state.update({key: m}), qos)
    for topic, target in [('/cmd_vel', commands), ('/simulation/cmd_vel', guarded)]:
        node.create_subscription(Twist, topic, lambda m, target=target:
                                 target.append((time.monotonic(), m)), 10)
    navigator = ActionClient(node, NavigateToPose, '/navigate_to_pose')
    planner = ActionClient(node, ComputePathToPose, '/compute_path_to_pose')
    active = None
    minimum_clearance = math.inf
    resume_samples = None

    def physical_pose():
        pose = truth['pose']
        return pose.position.x, pose.position.y, yaw(pose.orientation)

    def wait(predicate, timeout=60):
        nonlocal minimum_clearance
        deadline = time.monotonic()+timeout
        while not predicate():
            assert not errors and process.poll() is None, errors or 'Truth stream ended'
            assert time.monotonic() < deadline, 'Timed out waiting for navigation behavior'
            rclpy.spin_once(node, timeout_sec=.02)
            if 'pose' in truth:
                pose = physical_pose()
                gap = clearance(pose)
                assert gap > 0, 'Transport footprint intersected the central column'
                minimum_clearance = min(minimum_clearance, gap)
                if resume_samples is not None:
                    resume_samples.append(pose)

    def complete(future, timeout=60):
        wait(future.done, timeout)
        return future.result()

    def observe(duration):
        end = time.monotonic()+duration
        wait(lambda: time.monotonic() >= end, duration+2)

    def stopped():
        observe(1.5)
        assert guarded[-1][1] == Twist(), 'Guarded velocity is not zero'
        assert abs(state['odom'].twist.twist.linear.x) < .01
        assert abs(state['odom'].twist.twist.angular.z) < .02
        before = physical_pose()
        observe(1)
        after = physical_pose()
        assert math.dist(before[:2], after[:2]) < .02, 'Stopped translation drift'
        assert abs(angle_delta(before[2], after[2])) < .02, 'Stopped rotation drift'

    def pose_message(target):
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = node.get_clock().now().to_msg()
        pose.pose.position.x, pose.pose.position.y = float(target[0]), float(target[1])
        pose.pose.orientation.z = math.sin(target[2]/2)
        pose.pose.orientation.w = math.cos(target[2]/2)
        return pose

    def send_goal(target):
        goal = NavigateToPose.Goal()
        goal.pose = pose_message(target)
        handle = complete(navigator.send_goal_async(goal))
        assert handle.accepted, 'Navigation goal rejected'
        return handle

    def drive(publisher):
        command = Twist()
        command.linear.x = .2
        end = time.monotonic()+1.5
        while time.monotonic() < end:
            publisher.publish(command)
            observe(.05)
        assert state['odom'].twist.twist.linear.x > .15, 'Stopping test never reached motion'

    def pause(value):
        response = subprocess.run(['gz', 'service', '-s', WORLD+'control',
                                   '--reqtype', 'gz.msgs.WorldControl',
                                   '--reptype', 'gz.msgs.Boolean',
                                   '--timeout', '2000', '--req', f'pause: {str(value).lower()}'],
                                  check=True, capture_output=True, text=True, timeout=5)
        assert 'data: true' in response.stdout, 'Gazebo rejected pause/resume'

    try:
        required = ['odom', 'cloud', 'scan', 'imu', 'map', 'local', 'global']
        wait(lambda: all(k in state for k in required)
             and 'pose' in truth and bool(guarded))
        assert math.dist(physical_pose()[:2], (1, 1)) < .05, 'Use a fresh default arena spawn'
        for name in SERVERS:
            client = node.create_client(GetState, f'/{name}/get_state')
            assert client.wait_for_service(timeout_sec=30), name
            wait(lambda: complete(client.call_async(GetState.Request())).current_state.id == 3)
            node.destroy_client(client)
        stamp = Time.from_msg(state['scan'].header.stamp)
        wait(lambda: all(buffer.can_transform('map', frame, stamp)
                         for frame in ['base_link', 'unilidar_lidar', 'unilidar_imu']), 5)
        assert state['cloud'].height > 1 and state['scan'].header.stamp.sec > 0
        assert state['map'].data.count(0) > 500
        for key in ['map', 'local', 'global']:
            wait(lambda: occupied_near(state[key], 2.15, 1.5))
        names = [p.node_name for p in node.get_publishers_info_by_topic('/cmd_vel')]
        assert names == ['velocity_smoother'], names
        client = node.create_client(GetParameters, '/planner_server/get_parameters')
        request = GetParameters.Request(names=['planner_plugins', 'GridBased.plugin'])
        values = complete(client.call_async(request)).values
        assert list(values[0].string_array_value) == ['GridBased']
        assert values[1].string_value == 'nav2_smac_planner::SmacPlannerLattice'
        node.destroy_client(client)
        assert navigator.wait_for_server(timeout_sec=30)
        assert planner.wait_for_server(timeout_sec=30)
        request = ComputePathToPose.Goal(goal=pose_message(GOALS[0]), planner_id='GridBased')
        handle = complete(planner.send_goal_async(request))
        assert handle.accepted
        result = complete(handle.get_result_async())
        assert result.status == GoalStatus.STATUS_SUCCEEDED, result
        path = result.result.path.poses
        assert len(path) > 20
        detour = max(abs(2.6*p.pose.position.x-4*p.pose.position.y)/math.hypot(4, 2.6)
                     for p in path)
        assert detour > .6, 'Planner did not detour around the observed central column'
        report['planned_detour_m'] = detour
        initial_stamp = state['cloud'].header.stamp
        for target in GOALS:
            record = {'target': target, 'passed': False}
            report['goals'].append(record)
            started = time.monotonic()
            active = send_goal(target)
            result = complete(active.get_result_async(), args.timeout)
            active = None
            record.update(status=result.status, error_code=result.result.error_code,
                          wall_seconds=round(time.monotonic()-started, 2))
            assert result.status == GoalStatus.STATUS_SUCCEEDED, record
            stopped()
            physical = physical_pose()
            estimated = buffer.lookup_transform('map', 'base_link', Time()).transform
            error = math.dist(physical[:2], (target[0]+1, target[1]+1))
            map_error = math.hypot(estimated.translation.x-target[0],
                                   estimated.translation.y-target[1])
            heading_error = abs(angle_delta(physical[2], target[2]))
            record.update(physical_error_m=error, map_error_m=map_error,
                          heading_error_rad=heading_error)
            assert error < .35 and map_error < .25 and heading_error < .3, record
            record['passed'] = True
            print(json.dumps(record), flush=True)
        assert state['cloud'].header.stamp.sec > initial_stamp.sec, 'Sensor time stopped advancing'
        before = physical_pose()
        active = send_goal((2, 0, 0))
        pending = active.get_result_async()
        wait(lambda: math.dist(physical_pose()[:2], before[:2]) > .2, 40)
        assert not pending.done(), 'Cancellation test goal already completed'
        assert complete(active.cancel_goal_async()).goals_canceling
        assert complete(pending, 10).status == GoalStatus.STATUS_CANCELED
        active = None
        stopped()
        upstream = node.create_publisher(Twist, '/navigation/cmd_vel', 10)
        wait(lambda: upstream.get_subscription_count() > 0)
        drive(upstream)
        speed = state['odom'].twist.twist.linear.x
        try:
            pause(True)
            lost = time.monotonic()
            wait(lambda: commands[-1][0] > lost and commands[-1][1] == Twist(),
                 args.nav_cmd_timeout+1.5)
            wait(lambda: guarded[-1][0] > lost and guarded[-1][1] == Twist(), .15)
            observe(.2)
            frozen = physical_pose()
            resume_samples = []
            resumed = time.monotonic()
        finally:
            pause(False)
            node.destroy_publisher(upstream)
        stopped()
        distance = max(math.dist(p[:2], frozen[:2]) for p in resume_samples)
        assert distance <= speed**2/(2*.5)+.02, 'Excessive resumed translation beyond braking'
        assert max(abs(angle_delta(p[2], frozen[2])) for p in resume_samples) < .03
        resume_samples = None
        assert all(m == Twist() for at, m in guarded if at > resumed), 'Replayed movement command'
        report['resume_braking_m'] = distance
        manager = node.create_client(
            ManageLifecycleNodes, '/lifecycle_manager_navigation/manage_nodes')
        request = ManageLifecycleNodes.Request(command=ManageLifecycleNodes.Request.PAUSE)
        assert complete(manager.call_async(request)).success
        node.destroy_client(manager)
        publisher = node.create_publisher(Twist, '/cmd_vel', 10)
        wait(lambda: publisher.get_subscription_count() > 0)
        drive(publisher)
        lost = time.monotonic()
        wait(lambda: guarded[-1][0] > lost and guarded[-1][1] == Twist(), args.cmd_timeout+1)
        expiry = guarded[-1][0]-lost
        assert args.cmd_timeout-.12 <= expiry <= args.cmd_timeout+.15, expiry
        stopped()
        node.destroy_publisher(publisher)
        report.update(passed=True, public_expiry_seconds=expiry,
                      minimum_column_clearance_m=minimum_clearance)
        print('PASS: State Lattice, three goals, column avoidance, sensing/TF/SLAM, cancellation, '
              'paused-input expiry/resume and public-command expiry.', flush=True)
    except BaseException as error:
        report['error'] = f'{type(error).__name__}: {error}'
        raise
    finally:
        if active is not None:
            try:
                cancellation = active.cancel_goal_async()
                rclpy.spin_until_future_complete(node, cancellation, timeout_sec=3)
            except Exception as error:
                report['cleanup_error'] = str(error)
        process.terminate()
        try:
            process.wait(timeout=3)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait(timeout=3)
        reader.join(timeout=3)
        navigator.destroy()
        planner.destroy()
        listener.unregister()
        node.destroy_node()
        rclpy.try_shutdown()
        output.write_text(json.dumps(report, indent=2))


if __name__ == '__main__':
    main()
