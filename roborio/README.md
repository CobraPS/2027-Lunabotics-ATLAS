# ATLAS RoboRIO

C++ WPILib 2026 application for RoboRIO 2.0. This is a standalone Gradle
project; ROS 2 and the ROS devcontainer are not required.

## Setup

- Install the [WPILib 2026 development environment](https://docs.wpilib.org/en/stable/docs/zero-to-robot/step-2/wpilib-setup.html).
  This project uses WPILib/GradleRIO **2026.2.1**.
- Use **JDK 21**. Set `JAVA_HOME` and your terminal's Java path to that
  installation if another version is the default.
- Install a desktop C++ compiler: Linux `build-essential`, macOS Xcode
  command-line tools, or Windows Visual Studio 2022 C++ desktop tools.
- **CTRE Phoenix 6 26.3.0** and WPILib New Commands are configured in
  `vendordeps/`; Gradle downloads them automatically. Initial setup needs
  Internet access. A separate Gradle installation is unnecessary.

On Ubuntu/Debian, install the native compiler if needed:

```bash
sudo apt-get update
sudo apt-get install -y build-essential
```

From the repository root, verify Java 21 and install the RoboRIO cross-compiler:

```bash
cd roborio
java -version
./gradlew installRoboRioToolchain
```

Use `gradlew.bat` instead of `./gradlew` on Windows. Run all commands below
from `roborio/`.

## Build and test

```bash
./gradlew build
./gradlew check
```

`build` compiles RoboRIO and desktop debug/release binaries and runs the
automated tests. `check` runs the test/check tasks, reusing current results.
Neither requires connected hardware or deploys to a controller. Test XML
reports are under `build/test-results/frcUserProgramTest/`.

## Desktop simulation

```bash
./gradlew simulateNative
# Without the simulation GUI:
./gradlew simulateNative -Pheadless
```

Use the GUI's Driver Station controls to change modes; stop simulation with
Ctrl-C. The default configuration has no motors or drive commands, so enabling
produces no motion.

## Configuration

- **Motors:** Add confirmed `MotorConfig` entries to `constants::kMotors` in
  `src/main/include/Constants.h` and update the array size. Choose CAN IDs
  (0..62), bus names, and finite duty-cycle limits within [-1, 1] that include
  zero. Default limits are [0, 0]. Desktop simulation requires unique IDs
  across all buses.
- **Watchdog timeout:** Defaults to **250 ms**. Override the `RobotContainer`
  constructor's `driveCommandTimeout` argument with a positive, finite chrono
  duration, such as `std::chrono::milliseconds{100}`.
- **Hardware use:** The team number defaults to the unassigned placeholder 0
  in `build.gradle`. Confirm the real team number, wiring, motor assignments,
  inversion, and current/output limits before deployment. Physical hardware
  behavior remains unvalidated.

The watchdog is not yet integrated with NetworkTables reception (#11) or
drive mode control (#34). End-to-end communication-loss protection remains
incomplete; ordinary motor output APIs do not apply the watchdog automatically.
