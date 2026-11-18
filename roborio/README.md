# ATLAS RoboRIO foundation

A C++ command-based WPILib 2026 application for RoboRIO 2.0. This is an
independent Gradle project, not a ROS 2 package. `COLCON_IGNORE` keeps it out
of root-level `colcon` discovery; it needs neither ROS nor the ROS devcontainer.

## Environment and dependencies

| Component | Pinned version / requirement |
| --- | --- |
| WPILib / GradleRIO | 2026.2.1 |
| Gradle wrapper | 8.11 (distribution SHA-256 checked) |
| CTRE Phoenix 6 | 26.3.0, official 2026 vendordep committed under `vendordeps/` |
| WPILib New Commands | Official 2026 vendordep; resolves with WPILib version |
| Java | JDK 21 (set `JAVA_HOME` if another version is your default) |
| Desktop compiler | Linux `build-essential`, macOS Xcode command-line tools, or Windows Visual Studio 2022 C++ desktop tools |
| RoboRIO compiler | WPILib toolchain installed by the task below or WPILib 2026 installer |

The Gradle files and wrapper are adapted from the official
[WPILib VS Code 2026.2.1 template](https://github.com/wpilibsuite/vscode-wpilib/tree/v2026.2.1/vscode-wpilib/resources/gradle).
Use the [WPILib 2026 development environment](https://docs.wpilib.org/en/stable/docs/zero-to-robot/step-2/wpilib-setup.html)
for the supported editor/toolchain setup. A standalone JDK 21 plus native
compiler also works with the wrapper. A system Gradle installation is unnecessary.
Initial builds require Internet access to Gradle, WPILib, Maven Central, GitHub,
and CTRE repositories. Dependencies and the cross-compiler are cached in the
user's Gradle directory. GradleRIO 2026 uses the toolchain labeled **2025**;
that is expected, not a request to downgrade WPILib.

From the repository root:

```bash
cd roborio
java -version
./gradlew installRoboRioToolchain
./gradlew build
```

On Windows use `gradlew.bat`. `build` compiles RoboRIO and desktop debug/release
binaries and runs the desktop GoogleTest suite. It does not deploy or require
a connected controller. No team number is needed to build: the Gradle target's
zero is an unassigned placeholder, not the ATLAS team number. Deployment,
controller imaging, firmware selection, and physical motor validation are
outside this foundation's scope; confirm the real team/network/hardware
configuration before any later deployment work.

## Architecture and data flow

| Path | Responsibility |
| --- | --- |
| `src/main/cpp/Robot.cpp` and `include/Robot.h` | `TimedRobot` entry point; runs `CommandScheduler` every cycle and stops outputs on disable/mode entry |
| `src/main/cpp/RobotContainer.cpp` and `include/RobotContainer.h` | Validates hardware assignments, owns independently reusable subsystems, provides `StopAll()` |
| `src/main/include/Constants.h` | Hardware configuration list, deliberately empty until assignments are confirmed |
| `src/main/{cpp,include}/subsystems/TalonFXSubsystem.*` | One Phoenix Talon FX, duty-cycle policy, stop/disable handling, and command factory |
| `src/test/cpp/` | HAL/Driver Station and Phoenix software simulation tests |
| `vendordeps/` | Pinned Phoenix 6 and command framework dependency manifests |
| `build.gradle`, `settings.gradle`, `gradle/`, `gradlew*` | GradleRIO build, desktop simulation, tests, and wrapper |

`RunOutput(value)` creates a command requiring its motor subsystem. It calls
`SetOutput(value)` while scheduled and `Stop()` when ended/interrupted.
`SetOutput` validates robot state and input, clamps the request, then sends
Phoenix `DutyCycleOut` with FOC disabled (no Pro license required). There are
no input bindings or default motion commands. Direct calls must run on the
robot thread, and future commands must require the subsystem they control.

`MotorConfig` requires an explicit CAN ID (0..62); its default -1 is invalid.
It also accepts a bus name and independent minimum and
maximum duty cycles. Limits must be finite, include zero, and lie in [-1, 1].
Defaults are [0, 0], so motion requires an explicit choice. For illustration
only, `MotorConfig{20, "rio", -0.2, 0.3}` permits -20% through +30% output at
example CAN ID 20; it is not an ATLAS assignment. Add reviewed configurations
to `constants::kMotors` with the corresponding array size. No subsystem logic
needs to change with motor count or IDs. The container rejects duplicate
(bus, ID) pairs, and in desktop simulation rejects repeated IDs even across
buses because Phoenix simulation uses one virtual bus.

## Safety behavior and limits of this foundation

- Startup explicitly requests neutral before and after applying deterministic
  Phoenix defaults with the configured peak duty-cycle bounds.
- NaN and either infinity stop the motor and return `false`; finite values,
  including values outside [-1, 1], are clamped to the configured bounds.
- `Stop()` clears the software request and sends `NeutralOut`; a zero request
  does the same. Neutral is not a guarantee that a mechanism physically brakes.
- Requests while disabled are rejected and cleared immediately. Subsystem
  `Periodic()` clears requests while disabled, even without a command, and
  `DisabledInit`/`DisabledPeriodic` stop all configured motors. Re-enabling
  requires a new request or newly scheduled command.
- Failed Phoenix configuration locks out nonzero requests until the subsystem
  is reconstructed. A control/neutral request failure clears the stored request
  and inhibits motion. Only a successful neutral request while disabled can
  restore readiness after a control fault, and only if configuration succeeded.
  Recovery itself never requests motion; re-enabling still requires a new
  request or newly scheduled command. Sending neutral is best effort if CAN fails;
  a successful API return is not independent proof of physical motor state.
- All mode-entry hooks cancel scheduled commands before stopping outputs,
  including autonomous entry. WPILib does not automatically cancel commands
  when switching between enabled modes. There are no default motion commands.
- `GetRequestedOutput()` is the software request; `GetMeasuredOutput()` is
  asynchronous device telemetry. Neither proves physical motion or wiring.

Duty-cycle bounds are not mechanism current limits, calibrated speed control,
or a complete hardware safety design. Phoenix factory defaults (including
inversion and neutral behavior) need review alongside current limits, wiring,
and mechanical constraints before physical use. The project assumes no final
motor count, gearing, inversion assignments, or CAN wiring. It implements no
NetworkTables command reception (#11), drivetrain mixing, closed-loop tuning,
or independent drive-command watchdog. While enabled, an accepted direct
request persists until replaced/stopped; command freshness belongs to the
separate receiver/watchdog work.

## Testing

```bash
./gradlew build
./gradlew check
```

`check` runs the desktop GoogleTest tasks without rebuilding unrelated target
artifacts that are already current. Use `./gradlew tasks --all` to see individual
native test tasks, or `./gradlew clean build` for a clean rebuild. Native test
XML reports are under `build/test-results/frcUserProgramTest/`, separated by
platform and debug/release variant.

Tests initialize WPILib HAL, attach a simulated Driver Station, and construct
real Phoenix software-simulated Talon FX devices. They check initialization,
asymmetric positive/negative bounds, interior and extreme finite values,
NaN/infinities after motion, zero/explicit stopping, disabled-request rejection,
scheduler-driven disable/re-enable, command requirements/cancellation,
configuration validation, empty hardware configuration, and duplicate IDs.
Additional regressions inject Phoenix result codes into the private status
handlers to verify disabled-only recovery and permanent failed-configuration
lockout. These test software decisions, not physical CAN faults. Mode-transition
regressions invoke the actual robot hooks with scheduled motor commands.
Tests assert exact software requests and check simulated device duty-cycle
bounds/direction and stopping, allowing bounded time for asynchronous Phoenix
updates. Default current limiting can reduce simulated duty at stall; it is
kept enabled. No CAN adapter, RoboRIO, motor, Phoenix Pro license, ROS
installation, or display is required.

The dedicated [RoboRIO workflow](../.github/workflows/roborio-build.yml) uses
Ubuntu 24.04, Java 21, the native compiler, and the WPILib cross-compiler. It runs
`build` (including tests) on relevant PRs/main pushes and manual dispatch, and
uploads test results. The existing ROS 2 workflow is independent and unchanged.

## Desktop simulation

```bash
./gradlew simulateNative
# Headless, without the Sim GUI extension:
./gradlew simulateNative -Pheadless
```

Stop the running simulator with Ctrl-C. Use the GUI's Driver Station controls
to switch disabled/enabled modes. The default empty configuration has no
motors and enabling produces no motion. Automated tests instantiate example
virtual motors to exercise actual Phoenix control and disable behavior.
`SimulationPeriodic()` supplies simulated battery voltage; no mechanism physics,
encoder model, gearing, or rover dynamics are invented here. This validates
requests and simulated device responses, not physical drivetrain performance.

## Official references

- [WPILib project creation](https://docs.wpilib.org/en/stable/docs/software/vscode-overview/creating-robot-program.html)
- [WPILib desktop simulation](https://docs.wpilib.org/en/stable/docs/software/wpilib-tools/robot-simulation/introduction.html)
- [GradleRIO tasks](https://docs.wpilib.org/en/stable/docs/software/advanced-gradlerio/gradlew-tasks.html)
- [Phoenix 6 FRC installation](https://v6.docs.ctr-electronics.com/en/stable/docs/installation/installation-frc.html)
- [Phoenix 6 open-loop requests](https://v6.docs.ctr-electronics.com/en/stable/docs/api-reference/device-specific/talonfx/open-loop-requests.html)
- [Phoenix 6 simulation](https://v6.docs.ctr-electronics.com/en/stable/docs/api-reference/simulation/simulation-intro.html)
- [Phoenix hardware-free unit testing](https://v6.docs.ctr-electronics.com/en/stable/docs/api-reference/wpilib-integration/unit-testing.html)
