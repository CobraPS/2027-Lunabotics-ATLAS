# ATLAS Navigation

The `atlas_navigation` package provides a standalone simulation of the ATLAS rover using ROS 2 Jazzy and Gazebo Harmonic. It includes a four-wheel skid-steer rover model, simulated Unitree L2 LiDAR, online mapping with SLAM Toolbox, and goal-based navigation with Nav2.

The simulation runs without physical rover hardware or the RoboRIO bridge.

## Requirements

Use the repository's development container (provides ROS 2 Jazzy, Gazebo Harmonic, `ros_gz`, Nav2, SLAM Toolbox, `xacro`, `sensor_msgs_py`).

All commands below run from the **ROS 2 workspace root**.

## Quick Start: Autonomous Navigation

**1. Build and start the simulation:**
```bash
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash

export ROS_DOMAIN_ID=79
export GZ_PARTITION=atlas_provisional_test

ros2 launch atlas_navigation navigation_simulation.launch.py
```

This starts Gazebo, the simulated rover and LiDAR, SLAM Toolbox, and Nav2. Headless by default. SLAM builds a map from sensor data; no map file is loaded.

**2. Open a second terminal with the same environment:**
```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=79
export GZ_PARTITION=atlas_provisional_test
```

**3. Verify Nav2 is active and a map exists:**
```bash
ros2 lifecycle get /bt_navigator
ros2 topic echo /map --once --field info
```
Wait for `/bt_navigator` to report `active` and the map topic to respond.

**4. Send a navigation goal:**
```bash
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  '{pose: {header: {frame_id: map}, pose: {position: {x: 4.0, y: 2.6}, orientation: {w: 1.0}}}}' \
  --feedback
```

The rover spawns at the arena starting zone facing +X; the initial map/odometry origin is at the rover, so goals are expressed in the **map frame**. The simulated arena is a fixed illustrative layout with a central column that blocks the straight route. Choose goals with clearance for the full rover footprint and final heading.

Cancel all active goals:
```bash
ros2 service call /navigate_to_pose/_action/cancel_goal action_msgs/srv/CancelGoal '{}'
```

Stop the simulation with **Ctrl+C** in the launch terminal.

## Visualization

**Gazebo GUI** (attach to running simulation):
```bash
gz sim -g
```

**RViz2:**
```bash
rviz2
```

## Basic Simulation (Manual Control)

To test motion and sensors without Nav2:
```bash
ros2 launch atlas_navigation simulation.launch.py
```

In a second terminal (same environment), send a forward command:
```bash
ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.25}}'
```

Stop with **Ctrl+C**; the watchdog then brings the rover to a stop. For immediate stop:
```bash
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist '{}'
```

Do not manually publish to `/cmd_vel` while the autonomous navigation launch is running; its watchdog expires public commands. Stop autonomous navigation first, then use `simulation.launch.py` for manual control.

Inspect sensor output:
```bash
ros2 topic echo /odom --once
ros2 topic echo /unilidar/cloud --once --field header
ros2 topic echo /scan --once --field header
```

## Testing

Unit tests, linting, and model/config validation run in CI. The simulation checks below require a running simulation and are not part of CI.

### Navigation Integration Verification

```bash
# Terminal 1: start simulation
ros2 launch atlas_navigation navigation_simulation.launch.py

# Terminal 2: run verification (same ROS_DOMAIN_ID, GZ_PARTITION)
python3 src/atlas_navigation/test/verify_navigation.py --output /tmp/verify_report.json
```

### Shutdown Verification

```bash
python3 src/atlas_navigation/test/verify_shutdown.py --navigation --log /tmp/shutdown.log
```
