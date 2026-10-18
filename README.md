# 2027-Lunabotics-ATLAS
## Development Environment
1. Install Docker.
2. Install official Microsoft VS Code.
3. Install the Microsoft Dev Containers extension in VS Code.
4. Clone the repository.
5. Open the repository in VS Code.
6. Run `Dev Containers: Reopen in Container`.

The repository root is the ROS 2 workspace. ROS packages should be placed inside the `src/` directory.

To verify the development environment, open a terminal inside the dev container and run:

```bash
ros2 --help
colcon build
```

## RoboRIO Bridge

The `atlas_roborio_bridge` ROS 2 package provides a NetworkTables client for communication between the Jetson and RoboRIO.

The bridge currently supports:
- Connecting to a configurable NetworkTables server.
- Monitoring and reporting NetworkTables connection state changes.
- Publishing forward/reverse and turning drive command values.
- Remaining operational during NetworkTables disconnections and automatically reconnecting when the server becomes available.
- Subscribing to ROS 2 `/cmd_vel` messages and forwarding valid drive commands to NetworkTables.

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
