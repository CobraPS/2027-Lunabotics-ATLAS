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

The `atlas_roborio_bridge` package starts a NetworkTables client on the Jetson and reports its connection state to the RoboRIO server. ROS 2 topic and command exchange are not implemented yet.

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

Replace `192.0.2.10` with the RoboRIO NetworkTables server address.

If the NetworkTables server is unavailable, the bridge remains running and reports the disconnected state.
