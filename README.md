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
