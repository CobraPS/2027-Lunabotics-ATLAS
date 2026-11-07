# 2027-Lunabotics-ATLAS

ROS 2 Jazzy workspace for the SDSU Lunabotics ATLAS rover. This repository provides the navigation simulation, LiDAR driver, RoboRIO bridge, and bringup launch files.

## Development Environment

1. Install Docker.
2. Install a Dev Container tool (VS Code Dev Containers extension or [Dev Container CLI](https://github.com/devcontainers/cli)).
3. Clone the repository and open its root directory.
4. Select and start a dev container configuration:

| Configuration | File | Use Case |
|---------------|------|----------|
| **Default** | `.devcontainer/devcontainer.json` | Headless ROS 2 development, CI, non-GUI work |
| **Linux GUI - X11** | `.devcontainer/gui/devcontainer.json` | Gazebo/RViz2 on X11 or rootless XWayland (niri, sway, Hyprland, KDE X11) |
| **Linux GUI - Wayland** | `.devcontainer/gui-wayland/devcontainer.json` | Gazebo/RViz2 on native Wayland (KDE Plasma Wayland, GNOME) |

With the Dev Container CLI:
```bash
# X11 / rootless XWayland
devcontainer up --workspace-folder "$PWD" --config .devcontainer/gui/devcontainer.json
devcontainer exec --workspace-folder "$PWD" --config .devcontainer/gui/devcontainer.json bash

# Native Wayland
devcontainer up --workspace-folder "$PWD" --config .devcontainer/gui-wayland/devcontainer.json
devcontainer exec --workspace-folder "$PWD" --config .devcontainer/gui-wayland/devcontainer.json bash
```

## Build the Workspace

```bash
colcon build --symlink-install
source install/setup.bash
```

Each new terminal must source both setup files:
```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
```

For non-container development, install dependencies first:
```bash
rosdep install --from-paths src --ignore-src -r -y
```

## Navigation Simulation

See [ATLAS Navigation](src/atlas_navigation/README.md) for the standalone Gazebo, SLAM Toolbox, and Nav2 simulation, goal commands, visualization, launch arguments, and verification. It uses provisional geometry and runs independently of physical rover bringup.

## ATLAS Bringup

The `atlas_bringup` package provides the standard launch entry point for integrated ATLAS ROS 2 components (currently the RoboRIO bridge).

Start all integrated components:
```bash
ros2 launch atlas_bringup atlas.launch.py nt_server:=10.0.0.2
```

The `nt_server` argument defaults to an empty string, so no NetworkTables server is configured until you supply one. The example `10.0.0.2` is a documentation-only placeholder; replace it with the confirmed RoboRIO NetworkTables address.

## Unitree L2 LiDAR

The Unitree L2 driver (vendored in `src/unilidar_sdk2/`) can launch independently of the RoboRIO bridge.

Launch without RViz2 (headless default):
```bash
ros2 launch atlas_bringup unitree_l2.launch.py
```

Enable RViz2 (requires GUI environment):
```bash
ros2 launch atlas_bringup unitree_l2.launch.py use_rviz:=true
```

Default provisional addresses (not validated on hardware):
- `lidar_ip:=10.0.9.3` (sensor)
- `local_ip:=10.0.9.1` (host Ethernet)

Override as needed:
```bash
ros2 launch atlas_bringup unitree_l2.launch.py lidar_ip:=192.0.2.20 local_ip:=192.0.2.10
```

**Topic name note:** The vendor driver defaults to `unilidar/cloud` and `unilidar/imu` (no leading slash). The ATLAS simulation publishes `/unilidar/cloud` and `/unilidar/imu` (with leading slash). Verify topic names on physical hardware.

## RoboRIO Bridge

The `atlas_roborio_bridge` package provides a NetworkTables client for Jetson↔RoboRIO communication.

### NetworkTables Drive Command Interface

The bridge publishes two `double` values under the `SmartDashboard` table:

| NetworkTables Topic | Description | Provisional Units |
|---------------------|-------------|-------------------|
| `SmartDashboard/Jetson/DriveForward` | Forward/reverse drive command | m/s |
| `SmartDashboard/Jetson/DriveTurn` | Turning drive command | rad/s |

The bridge forwards ROS 2 `/cmd_vel` (`linear.x`, `angular.z`) without scaling.

### Build and Run

```bash
colcon build --symlink-install
source install/setup.bash
ros2 run atlas_roborio_bridge roborio_bridge --ros-args -p nt_server:=10.0.0.2
```

Replace `10.0.0.2` with the actual RoboRIO NetworkTables server address. The bridge remains running and reports connection state if the server is unavailable.

### Testing

Connection loss and reconnection have been tested with a local NetworkTables server (no physical RoboRIO).

### Current Limitations

- Physical RoboRIO integration not verified
- RoboRIO receiver's expected NetworkTables topics/units unconfirmed
- Jetson-side stale-command timeout implemented; RoboRIO-side validation outstanding
- Physical drivetrain integration not tested