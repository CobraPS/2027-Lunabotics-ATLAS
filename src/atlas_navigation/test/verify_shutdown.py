"""Start a fresh isolated simulation and verify that SIGINT leaves no child processes."""

import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import time
import uuid

import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


def session_processes(session):
    """Find live members of the launch's private Linux process session."""
    members = []
    for path in Path('/proc').iterdir():
        if path.name.isdigit():
            try:
                if os.getsid(int(path.name)) == session:
                    state = (path / 'stat').read_text().rsplit(')', 1)[1].split()[0]
                    if state != 'Z':
                        members.append(int(path.name))
            except (ProcessLookupError, FileNotFoundError):
                pass
    return members


def main():
    """Require actual sensor output before testing normal launch teardown."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--navigation', action='store_true')
    parser.add_argument('--log', required=True)
    args = parser.parse_args()
    launch = 'navigation_simulation.launch.py' if args.navigation else 'simulation.launch.py'
    token = uuid.uuid4()
    original_domain = os.environ.get('ROS_DOMAIN_ID', '0')
    domain = 100 + token.int % 100
    if str(domain) == original_domain:
        domain = 100 + (domain - 99) % 100
    os.environ['ROS_DOMAIN_ID'] = str(domain)
    os.environ['GZ_PARTITION'] = 'atlas_shutdown_' + token.hex
    rclpy.init()
    node = rclpy.create_node('atlas_shutdown_verification')
    scans = []
    subscription = node.create_subscription(LaserScan, '/scan', scans.append,
                                            qos_profile_sensor_data)
    with open(args.log, 'w') as log:
        process = subprocess.Popen(['ros2', 'launch', 'atlas_navigation', launch],
                                   stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        try:
            deadline = time.monotonic() + 60
            while not scans or scans[-1].header.stamp.sec == 0:
                assert process.poll() is None, 'Launch exited before sensing became ready'
                assert time.monotonic() < deadline, 'No live scan before shutdown test'
                rclpy.spin_once(node, timeout_sec=.02)
            assert scans[-1].header.stamp.sec > 0
            if args.navigation:
                from lifecycle_msgs.srv import GetState
                from verify_navigation import SERVERS

                for name in SERVERS:
                    client = node.create_client(GetState, f'/{name}/get_state')
                    assert client.wait_for_service(timeout_sec=30), name
                    while True:
                        future = client.call_async(GetState.Request())
                        rclpy.spin_until_future_complete(node, future, timeout_sec=5)
                        assert future.done(), name
                        if future.result().current_state.id == 3:
                            break
                        assert time.monotonic() < deadline, f'{name} did not activate'
                    node.destroy_client(client)
            started = time.monotonic()
            process.send_signal(signal.SIGINT)
            process.wait(timeout=15)
            assert process.returncode == 0, f'Launch exited with {process.returncode}'
            # Root exit must not leave an orphan server in its private session.
            survivors = session_processes(process.pid)
            assert not survivors, f'Launch left live processes: {survivors}'
            print(json.dumps({'launch': launch, 'exit_code': process.returncode,
                              'shutdown_wall_seconds': round(time.monotonic() - started, 3),
                              'surviving_processes': survivors}), flush=True)
            print('PASS: sensing was live; launch shutdown left no child processes.')
        finally:
            if session_processes(process.pid):
                os.killpg(process.pid, signal.SIGKILL)
            process.wait(timeout=5)
            node.destroy_subscription(subscription)
            node.destroy_node()
            rclpy.try_shutdown()


if __name__ == '__main__':
    main()
