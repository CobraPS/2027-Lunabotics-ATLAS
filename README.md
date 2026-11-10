# 2027-Lunabotics-ATLAS

## Development Environment

1. Install Docker.
2. Install a tool that supports Dev Containers, or the [Dev Container CLI](https://github.com/devcontainers/cli).
3. Clone the repository and open its root directory.
4. Create and open a dev container using one of these configurations:

- **Default** (`.devcontainer/devcontainer.json`): general ROS 2 development and CI; no host display required.
- **Linux GUI** (`.devcontainer/gui/devcontainer.json`): optional RViz2 visualization and physical LiDAR testing on a Linux host with a local Docker Engine.

Both configurations use the existing Dockerfile, which includes RViz2.

Use your tool's supported method to select the configuration path; selection workflows vary between tools. With the Dev Container CLI installed, run these commands from the repository root to use the GUI configuration:

```bash
devcontainer up --workspace-folder "$PWD" --config .devcontainer/gui/devcontainer.json
devcontainer exec --workspace-folder "$PWD" --config .devcontainer/gui/devcontainer.json bash
```

For the default configuration, use `.devcontainer/devcontainer.json` in both commands. Changed settings apply to newly created containers, not existing ones.

The optional Linux GUI devcontainer supports graphical applications such as RViz2 and requires access to the host's X11 or Xwayland display. Run `rviz2` inside the GUI container to verify that a window appears.

For physical LiDAR testing that needs access to the host Ethernet interface, run `export ATLAS_DOCKER_NETWORK=host` before creating the GUI container. Confirm the host interface and LiDAR IP addresses separately. Bridge networking remains the default when this variable is unset.

The repository root is the ROS 2 workspace. ROS packages should be placed inside the `src/` directory.

To verify the development environment, open a terminal inside the dev container and run:

```bash
ros2 --help
colcon build
```

## ATLAS Bringup

The `atlas_bringup` package provides the standard launch entry point for starting the currently integrated ATLAS ROS 2 components. It currently starts the RoboRIO bridge. Additional sensors and autonomy components will be added as they are integrated.

Build and source the workspace:

```bash
colcon build --symlink-install
source install/setup.bash
```

Start all currently integrated ATLAS components:

```bash
ros2 launch atlas_bringup atlas.launch.py
```

To configure the NetworkTables server when launching ATLAS:

```bash
ros2 launch atlas_bringup atlas.launch.py nt_server:=192.0.2.10
```

`192.0.2.10` is a documentation-only example address, not the confirmed ATLAS RoboRIO address. Replace it after the team confirms the actual network configuration.

## Unitree L2 LiDAR

The Unitree L2 can launch independently of the RoboRIO bridge using the official `unitree_lidar_ros2` driver. Build and source the workspace as described in ATLAS Bringup before running these commands.

Launch the LiDAR driver without RViz2:

```bash
ros2 launch atlas_bringup unitree_l2.launch.py
```

RViz2 is disabled by default (`use_rviz:=false`) for headless operation. To enable visualization, use the Linux GUI environment described above and run:

```bash
ros2 launch atlas_bringup unitree_l2.launch.py use_rviz:=true
```

The default addresses are `lidar_ip:=10.0.9.3` for the sensor and `local_ip:=10.0.9.1` for the host. These ATLAS settings are provisional; physical sensor communication has not yet been validated.

Override the addresses as needed:

```bash
ros2 launch atlas_bringup unitree_l2.launch.py \
  lidar_ip:=192.0.2.20 local_ip:=192.0.2.10
```

These override values are documentation-only examples. Replace them with the sensor's configured IP and the host Ethernet interface's IP.

When sensor data is received, the driver publishes these default topics:

| Topic | ROS 2 Message Type |
|---|---|
| `/unilidar/cloud` | `sensor_msgs/msg/PointCloud2` |
| `/unilidar/imu` | `sensor_msgs/msg/Imu` |

Run the `atlas_bringup` automated tests:

```bash
colcon test --packages-select atlas_bringup
colcon test-result --verbose
```

## RoboRIO Bridge

The `atlas_roborio_bridge` ROS 2 package provides a NetworkTables client for communication between the Jetson and RoboRIO.

The bridge currently supports:

- Connecting to a configurable NetworkTables server.
- Monitoring and reporting NetworkTables connection state changes.
- Publishing forward/reverse and turning drive command values.
- Remaining operational during NetworkTables disconnections and automatically reconnecting when the server becomes available.
- Subscribing to ROS 2 `/cmd_vel` messages and forwarding valid drive commands to NetworkTables.

## Keyboard Teleoperation

Publishes `geometry_msgs/msg/Twist` commands to `/cmd_vel` using the official `teleop_twist_keyboard` package, which is installed in the dev container. It must be the only `/cmd_vel` publisher, because command arbitration with autonomous navigation is not implemented yet.

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

The initial speeds can be set without changing any code (defaults: `speed` 0.5, `turn` 1.0):

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p speed:=0.3 -p turn:=0.8
```

Keys: `i` forward, `,` reverse, `j` turn left, `l` turn right, `k` stop (any key without another function also stops). `q`/`z` change both speeds by 10%, `w`/`x` only the linear speed, `e`/`c` only the angular speed. `Ctrl+C` quits after publishing a final zero-velocity command, which does not replace the command timeout and failsafe mechanisms. The published commands can be observed with `ros2 topic echo /cmd_vel`.

### Test through the RoboRIO bridge

Open three terminals in the dev container.

1. Start a local NetworkTables server that prints the drive values (run from `/tmp`, stop with `Ctrl+C`):

```bash
cd /tmp
python3 - <<'EOF'
import time

import ntcore

inst = ntcore.NetworkTableInstance.getDefault()
inst.startServer()
forward = inst.getDoubleTopic('/SmartDashboard/Jetson/DriveForward').subscribe(0.0)
turn = inst.getDoubleTopic('/SmartDashboard/Jetson/DriveTurn').subscribe(0.0)
last = None
try:
    while True:
        current = (forward.get(), turn.get())
        if current != last:
            print(f'DriveForward={current[0]:.2f} DriveTurn={current[1]:.2f}')
            last = current
        time.sleep(0.05)
except KeyboardInterrupt:
    pass
EOF
```

2. Start the bridge: `ros2 run atlas_roborio_bridge roborio_bridge --ros-args -p nt_server:=127.0.0.1 -p cmd_timeout:=1.0`
3. Start the keyboard teleoperation with the command above.

With the default speeds, pressing `i` prints `DriveForward=0.50 DriveTurn=0.00` and pressing `j` prints `DriveForward=0.00 DriveTurn=1.00`. A key held down is repeated by the keyboard after a short delay, so `cmd_timeout` should be longer than that delay (for example `1.0`).

### NetworkTables Drive Command Interface

The bridge publishes two NetworkTables `double` values under the `SmartDashboard` table:

| NetworkTables Topic | Description | Provisional Units |
|---|---|---|
| `SmartDashboard/Jetson/DriveForward` | Forward/reverse drive command | Meters per second (m/s) |
| `SmartDashboard/Jetson/DriveTurn` | Turning drive command | Radians per second (rad/s) |

These topic names are based on the previous ATHENA Jetson implementation.

The interface provisionally assumes that ROS 2 `/cmd_vel` values (`linear.x` and `angular.z`) will be forwarded without scaling or conversion.

**Important:** The actual RoboRIO receiver implementation has not yet been verified. Topic names, expected units, and any necessary conversion must be confirmed against the RoboRIO source code before controlling physical hardware.

### Build and Run

Build and source the workspace:

```bash
colcon build
source install/setup.bash
```

Run the bridge with the RoboRIO NetworkTables server address:

```bash
ros2 run atlas_roborio_bridge roborio_bridge \
  --ros-args -p nt_server:=192.0.2.10
```

Replace `192.0.2.10` with the actual RoboRIO NetworkTables server address.

If the server is unavailable, the bridge remains running and reports its connection state.

### Testing

Run the package's automated tests:

```bash
colcon test --packages-select atlas_roborio_bridge
colcon test-result --verbose
```

The publishing method has also been verified using a local NetworkTables server without physical RoboRIO hardware. Connection loss and reconnection have been tested.

### Current Limitations

- Valid ROS 2 `/cmd_vel` messages are forwarded to NetworkTables, but end-to-end integration with the physical RoboRIO has not yet been verified.
- The RoboRIO receiver's expected NetworkTables topics and value units remain unverified.
- Safe handling of stale drive commands is addressed separately in Issue #12.
- Physical drivetrain integration has not yet been tested.
