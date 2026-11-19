#include <frc/DriverStation.h>
#include <frc/simulation/DriverStationSim.h>
#include <frc2/command/CommandScheduler.h>
#include <gtest/gtest.h>

#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <stdexcept>
#include <thread>

#include "DriveCommandWatchdog.h"
#include "Robot.h"
#include "subsystems/TalonFXSubsystem.h"

// Inject vendor results only at the status-handling boundary. The ordinary
// control path and device-output assertions still use Phoenix simulation.
class TalonFXSubsystemTestPeer {
 public:
  static void NeutralResult(TalonFXSubsystem& motor,
                            ctre::phoenix::StatusCode status) {
    motor.RecordNeutralResult(status);
  }

  static void ConfigurationResult(TalonFXSubsystem& motor,
                                  ctre::phoenix::StatusCode status) {
    motor.RecordConfigurationResult(status);
  }
};

namespace {
using namespace std::chrono_literals;

class TalonFXSubsystemTest : public testing::Test {
 protected:
  void SetUp() override {
    frc::sim::DriverStationSim::ResetData();
    frc::sim::DriverStationSim::SetDsAttached(true);
    SetEnabled(false);
    // Distinct virtual IDs avoid reusing a device during Phoenix teardown.
    static int nextId = 0;
    motor = std::make_unique<TalonFXSubsystem>(
        MotorConfig{nextId++, "rio", -0.25, 0.40});
    ASSERT_TRUE(motor->IsReady());
    motor->SimulationPeriodic();
    SetEnabled(true);
    std::this_thread::sleep_for(100ms);  // CTRE device startup/enable delay.
  }

  void TearDown() override {
    frc2::CommandScheduler::GetInstance().CancelAll();
    SetEnabled(false);
    motor.reset();
  }

  void SetEnabled(bool enabled) {
    frc::sim::DriverStationSim::SetEnabled(enabled);
    frc::sim::DriverStationSim::NotifyNewData();
    ASSERT_EQ(frc::DriverStation::IsEnabled(), enabled);
  }

  void Tick() {
    // Keep the simulated Driver Station attached, without altering enable
    // state.
    frc::sim::DriverStationSim::NotifyNewData();
    frc2::CommandScheduler::GetInstance().Run();
    std::this_thread::sleep_for(20ms);
  }

  void ExpectDeviceInRange(double lower, double upper) {
    // Phoenix signals are asynchronous; wait up to one second for fresh output.
    double actual = 0;
    for (int i = 0; i < 50; ++i) {
      Tick();
      actual = motor->GetMeasuredOutput();
      if (actual >= lower && actual <= upper) {
        return;
      }
    }
    EXPECT_GE(actual, lower);
    EXPECT_LE(actual, upper);
  }

  void ExpectDeviceOutput(double expected) {
    ExpectDeviceInRange(expected - 0.015, expected + 0.015);
  }

  std::unique_ptr<TalonFXSubsystem> motor;
};

TEST_F(TalonFXSubsystemTest, StartsAtZeroEvenWhenEnabled) {
  EXPECT_DOUBLE_EQ(motor->GetRequestedOutput(), 0.0);
  ExpectDeviceOutput(0.0);
}

TEST_F(TalonFXSubsystemTest, ClampsBothBoundsAndPreservesInteriorRequests) {
  for (auto [request, expected] :
       {std::pair{1.0, 0.4},
        {-1.0, -0.25},
        {0.15, 0.15},
        {-0.1, -0.1},
        {0.4, 0.4},
        {-0.25, -0.25},
        {std::numeric_limits<double>::max(), 0.4},
        {std::numeric_limits<double>::lowest(), -0.25}}) {
    ASSERT_TRUE(motor->SetOutput(request));
    EXPECT_DOUBLE_EQ(motor->GetRequestedOutput(), expected);
    // No rotor physics is modeled here. Phoenix current limiting may reduce
    // duty at simulated stall: require a nonzero response in the correct
    // direction, bounded by the exact software request checked above.
    if (expected > 0.0) {
      ExpectDeviceInRange(0.015, expected + 0.015);
    } else {
      ExpectDeviceInRange(expected - 0.015, -0.015);
    }
  }
}

TEST_F(TalonFXSubsystemTest, NonFiniteRequestsStopAnAlreadyMovingMotor) {
  for (double invalid : {std::numeric_limits<double>::quiet_NaN(),
                         std::numeric_limits<double>::infinity(),
                         -std::numeric_limits<double>::infinity()}) {
    ASSERT_TRUE(motor->SetOutput(0.2));
    ExpectDeviceOutput(0.2);
    EXPECT_FALSE(motor->SetOutput(invalid));
    EXPECT_DOUBLE_EQ(motor->GetRequestedOutput(), 0.0);
    ExpectDeviceOutput(0.0);
  }
}

TEST_F(TalonFXSubsystemTest, ExplicitStopAndZeroRequestNeutralizeMotor) {
  ASSERT_TRUE(motor->SetOutput(-0.2));
  ExpectDeviceOutput(-0.2);
  motor->Stop();
  motor->Stop();  // Idempotent.
  EXPECT_DOUBLE_EQ(motor->GetRequestedOutput(), 0.0);
  ExpectDeviceOutput(0.0);
  ASSERT_TRUE(motor->SetOutput(0.2));
  ExpectDeviceOutput(0.2);
  ASSERT_TRUE(motor->SetOutput(0.0));
  EXPECT_DOUBLE_EQ(motor->GetRequestedOutput(), 0.0);
  ExpectDeviceOutput(0.0);
}

TEST_F(TalonFXSubsystemTest, DisabledRequestsAreRejectedImmediately) {
  ASSERT_TRUE(motor->SetOutput(0.2));
  ExpectDeviceOutput(0.2);
  SetEnabled(false);
  EXPECT_FALSE(motor->SetOutput(0.3));
  EXPECT_DOUBLE_EQ(motor->GetRequestedOutput(), 0.0);
  ExpectDeviceOutput(0.0);
}

TEST_F(TalonFXSubsystemTest, SchedulerClearsDisabledRequestAndDoesNotReplayIt) {
  ASSERT_TRUE(motor->SetOutput(-0.2));
  ExpectDeviceOutput(-0.2);
  SetEnabled(false);
  Tick();
  EXPECT_DOUBLE_EQ(motor->GetRequestedOutput(), 0.0);
  ExpectDeviceOutput(0.0);
  SetEnabled(true);
  for (int i = 0; i < 6; ++i) {
    Tick();
    EXPECT_DOUBLE_EQ(motor->GetRequestedOutput(), 0.0);
    EXPECT_NEAR(motor->GetMeasuredOutput(), 0.0, 0.015);
  }
  ASSERT_TRUE(motor->SetOutput(0.2));  // A new request is required.
  ExpectDeviceOutput(0.2);
}

TEST_F(TalonFXSubsystemTest, CommandOwnsSubsystemAndStopsWhenCanceled) {
  auto command = motor->RunOutput(0.3);
  EXPECT_TRUE(command.HasRequirement(motor.get()));
  frc2::CommandScheduler::GetInstance().Schedule(command);
  ExpectDeviceOutput(0.3);
  command.Cancel();
  EXPECT_DOUBLE_EQ(motor->GetRequestedOutput(), 0.0);
  ExpectDeviceOutput(0.0);
}

TEST_F(TalonFXSubsystemTest,
       DisableCancelsCommandAndKeepsMotorStoppedOnEnable) {
  auto command = motor->RunOutput(0.3);
  frc2::CommandScheduler::GetInstance().Schedule(command);
  ExpectDeviceOutput(0.3);
  SetEnabled(false);
  Tick();
  EXPECT_FALSE(command.IsScheduled());
  EXPECT_DOUBLE_EQ(motor->GetRequestedOutput(), 0.0);
  ExpectDeviceOutput(0.0);
  SetEnabled(true);
  Tick();
  EXPECT_FALSE(command.IsScheduled());
  ExpectDeviceOutput(0.0);
}

TEST_F(TalonFXSubsystemTest, DisabledNeutralFailureNeedsDisabledRecovery) {
  ASSERT_TRUE(motor->SetOutput(0.2));
  ExpectDeviceOutput(0.2);
  SetEnabled(false);
  TalonFXSubsystemTestPeer::NeutralResult(*motor,
                                          ctre::phoenix::StatusCode::TxTimeout);
  EXPECT_FALSE(motor->IsReady());
  EXPECT_DOUBLE_EQ(motor->GetRequestedOutput(), 0.0);
  TalonFXSubsystemTestPeer::NeutralResult(*motor,
                                          ctre::phoenix::StatusCode::TxFailed);
  EXPECT_FALSE(motor->IsReady());

  // Enabling before neutral recovery must not restore an old request. Even a
  // successful periodic neutral retry while enabled cannot re-arm the motor.
  SetEnabled(true);
  Tick();
  EXPECT_FALSE(motor->IsReady());
  EXPECT_FALSE(motor->SetOutput(0.2));
  ExpectDeviceOutput(0.0);
  EXPECT_FALSE(motor->IsReady());

  SetEnabled(false);
  Tick();  // The real Phoenix simulated neutral request now succeeds.
  EXPECT_TRUE(motor->IsReady());
  SetEnabled(true);
  Tick();
  EXPECT_DOUBLE_EQ(motor->GetRequestedOutput(), 0.0);
  ExpectDeviceOutput(0.0);
  ASSERT_TRUE(motor->SetOutput(0.2));
  ExpectDeviceOutput(0.2);
}

TEST_F(TalonFXSubsystemTest,
       EnabledNeutralFailureCannotResumeScheduledCommand) {
  auto command = motor->RunOutput(0.2);
  frc2::CommandScheduler::GetInstance().Schedule(command);
  ExpectDeviceOutput(0.2);
  TalonFXSubsystemTestPeer::NeutralResult(*motor,
                                          ctre::phoenix::StatusCode::TxFailed);
  EXPECT_DOUBLE_EQ(motor->GetRequestedOutput(), 0.0);
  ExpectDeviceOutput(0.0);
  EXPECT_FALSE(motor->IsReady());
  EXPECT_TRUE(command.IsScheduled());

  SetEnabled(false);
  Tick();
  EXPECT_TRUE(motor->IsReady());
  EXPECT_FALSE(command.IsScheduled());
  SetEnabled(true);
  for (int i = 0; i < 6; ++i) {
    Tick();
    EXPECT_DOUBLE_EQ(motor->GetRequestedOutput(), 0.0);
  }
  ExpectDeviceOutput(0.0);
  frc2::CommandScheduler::GetInstance().Schedule(command);  // Explicit restart.
  ExpectDeviceOutput(0.2);
  command.Cancel();
}

TEST_F(TalonFXSubsystemTest, NeutralSuccessCannotMaskFailedConfiguration) {
  // Represents a failed constructor Apply result; neutral communication alone
  // says nothing about whether output limits were successfully configured.
  TalonFXSubsystemTestPeer::ConfigurationResult(
      *motor, ctre::phoenix::StatusCode::RxTimeout);
  EXPECT_FALSE(motor->IsReady());
  SetEnabled(false);
  for (int i = 0; i < 3; ++i) {
    Tick();
    EXPECT_FALSE(motor->IsReady());
  }
  SetEnabled(true);
  EXPECT_FALSE(motor->SetOutput(0.2));
  EXPECT_FALSE(motor->IsReady());
  EXPECT_DOUBLE_EQ(motor->GetRequestedOutput(), 0.0);
  ExpectDeviceOutput(0.0);
}

TEST_F(TalonFXSubsystemTest, AutonomousEntryCancelsEnabledModeMotorCommand) {
  Robot robot;
  auto command = motor->RunOutput(0.2);
  frc2::CommandScheduler::GetInstance().Schedule(command);
  ExpectDeviceOutput(0.2);
  frc::sim::DriverStationSim::SetAutonomous(true);
  frc::sim::DriverStationSim::NotifyNewData();
  ASSERT_TRUE(command.IsScheduled());  // WPILib does not cancel on mode change.

  robot.AutonomousInit();
  EXPECT_FALSE(command.IsScheduled());
  EXPECT_DOUBLE_EQ(motor->GetRequestedOutput(), 0.0);
  robot.RobotPeriodic();
  ExpectDeviceOutput(0.0);
}

TEST_F(TalonFXSubsystemTest,
       AutonomousEntryCancelsDisabledCapableMotorCommand) {
  Robot robot;
  SetEnabled(false);
  auto command = motor->RunOutput(0.2).IgnoringDisable(true);
  frc2::CommandScheduler::GetInstance().Schedule(command);
  Tick();
  ASSERT_TRUE(command.IsScheduled());
  EXPECT_DOUBLE_EQ(motor->GetRequestedOutput(), 0.0);
  frc::sim::DriverStationSim::SetAutonomous(true);
  SetEnabled(true);

  robot.AutonomousInit();
  EXPECT_FALSE(command.IsScheduled());
  robot.RobotPeriodic();
  EXPECT_DOUBLE_EQ(motor->GetRequestedOutput(), 0.0);
  ExpectDeviceOutput(0.0);
}

// Only Phoenix telemetry waits use wall time. All watchdog deadlines below
// use this controllable steady-clock time point, including scheduler tests.
class WatchedOutputTest : public TalonFXSubsystemTest {
 protected:
  void StepScheduler() {
    frc::sim::DriverStationSim::NotifyNewData();
    frc2::CommandScheduler::GetInstance().Run();
  }

  // Emulate only acceptance of a complete update, with no receiver or mode
  // policy. Rejected input never replaces the demand exposed to the command.
  bool AcceptDemand(std::uint64_t updateId, double demand, bool isValid = true) {
    if (!watchdog.RecordCommand(updateId, isValid && std::isfinite(demand))) {
      return false;
    }
    acceptedDemand = demand;
    return true;
  }

  DriveCommandWatchdog::TimePoint now{10s};
  RobotContainer container{constants::kMotors, 250ms,
                           [this] { return now; }};
  DriveCommandWatchdog& watchdog = container.GetDriveCommandWatchdog();
  double acceptedDemand = 0.0;
};

TEST_F(WatchedOutputTest, StartupCannotRequestOutputWithoutAValidCommand) {
  auto command = motor->RunWatchedOutput([] { return 0.2; }, watchdog);
  EXPECT_TRUE(command.HasRequirement(motor.get()));
  command.Schedule();
  Tick();
  EXPECT_DOUBLE_EQ(motor->GetRequestedOutput(), 0.0);
  EXPECT_FALSE(command.IsScheduled());
  ExpectDeviceOutput(0.0);
}

TEST_F(WatchedOutputTest, ExactDeadlineStopsOutputAndEndsTheCommand) {
  ASSERT_TRUE(watchdog.RecordCommand(1, true));
  auto command = motor->RunWatchedOutput([] { return 1.0; }, watchdog);
  command.Schedule();
  Tick();
  EXPECT_DOUBLE_EQ(motor->GetRequestedOutput(), 0.4);  // Existing limit.
  ExpectDeviceInRange(0.015, 0.415);
  now += 250ms - 1ns;
  Tick();
  EXPECT_DOUBLE_EQ(motor->GetRequestedOutput(), 0.4);
  now += 1ns;
  Tick();
  EXPECT_DOUBLE_EQ(motor->GetRequestedOutput(), 0.0);
  EXPECT_FALSE(command.IsScheduled());
  ExpectDeviceOutput(0.0);
  for (int i = 0; i < 3; ++i) {
    EXPECT_FALSE(watchdog.RecordCommand(1, true));
    command.Schedule();  // Even an explicit restart cannot bypass the gate.
    Tick();
    EXPECT_DOUBLE_EQ(motor->GetRequestedOutput(), 0.0);
    EXPECT_FALSE(command.IsScheduled());
  }
}

TEST_F(WatchedOutputTest, ExpiredExecutionCannotReissueOutputBeforeCommandEnd) {
  ASSERT_TRUE(watchdog.RecordCommand(1, true));
  auto command = motor->RunWatchedOutput([] { return 0.2; }, watchdog);
  // Inspect the Execute/End boundary explicitly: checking only the final
  // scheduler output would miss a nonzero request immediately cleared by End.
  command.get()->Initialize();
  command.get()->Execute();
  EXPECT_DOUBLE_EQ(motor->GetRequestedOutput(), 0.2);
  now += 250ms;
  motor->Stop();
  command.get()->Execute();
  EXPECT_DOUBLE_EQ(motor->GetRequestedOutput(), 0.0);
  EXPECT_TRUE(command.get()->IsFinished());
  command.get()->End(false);
  EXPECT_DOUBLE_EQ(motor->GetRequestedOutput(), 0.0);
}

TEST_F(WatchedOutputTest, RecoveryRequiresANewValidUpdateAndCommandScheduling) {
  ASSERT_TRUE(watchdog.RecordCommand(10, true));
  auto command = motor->RunWatchedOutput([] { return -0.2; }, watchdog);
  command.Schedule();
  Tick();
  ASSERT_DOUBLE_EQ(motor->GetRequestedOutput(), -0.2);
  now += 250ms;
  Tick();
  EXPECT_DOUBLE_EQ(motor->GetRequestedOutput(), 0.0);
  EXPECT_FALSE(command.IsScheduled());
  EXPECT_FALSE(watchdog.RecordCommand(10, true));
  EXPECT_FALSE(watchdog.RecordCommand(11, false));
  command.Schedule();
  Tick();
  EXPECT_DOUBLE_EQ(motor->GetRequestedOutput(), 0.0);
  ASSERT_TRUE(watchdog.RecordCommand(12, true));
  Tick();
  EXPECT_FALSE(command.IsScheduled());
  EXPECT_DOUBLE_EQ(motor->GetRequestedOutput(), 0.0);
  command.Schedule();
  Tick();
  EXPECT_DOUBLE_EQ(motor->GetRequestedOutput(), -0.2);
  command.Cancel();
  EXPECT_DOUBLE_EQ(motor->GetRequestedOutput(), 0.0);
}

TEST_F(WatchedOutputTest, NewUpdatesExtendValidityButInvalidUpdatesDoNot) {
  ASSERT_TRUE(watchdog.RecordCommand(1, true));
  auto command = motor->RunWatchedOutput([] { return 0.2; }, watchdog);
  command.Schedule();
  Tick();
  now += 200ms;
  ASSERT_TRUE(watchdog.RecordCommand(2, true));
  now += 200ms;
  EXPECT_FALSE(watchdog.RecordCommand(3, false));
  Tick();
  EXPECT_DOUBLE_EQ(motor->GetRequestedOutput(), 0.2);
  now += 50ms;
  Tick();
  EXPECT_DOUBLE_EQ(motor->GetRequestedOutput(), 0.0);
  EXPECT_FALSE(command.IsScheduled());
}

TEST_F(WatchedOutputTest, NewAcceptedZeroDemandStopsAnAlreadyScheduledCommand) {
  ASSERT_TRUE(AcceptDemand(1, 0.2));
  auto command = motor->RunWatchedOutput(
      [this] { return acceptedDemand; }, watchdog);
  command.Schedule();
  StepScheduler();
  ASSERT_DOUBLE_EQ(motor->GetRequestedOutput(), 0.2);

  now += 200ms;
  ASSERT_TRUE(AcceptDemand(2, 0.0));
  StepScheduler();
  EXPECT_TRUE(command.IsScheduled());
  EXPECT_TRUE(watchdog.IsActive());
  EXPECT_DOUBLE_EQ(motor->GetRequestedOutput(), 0.0);
  now += 249ms;
  StepScheduler();
  EXPECT_TRUE(command.IsScheduled());
  EXPECT_DOUBLE_EQ(motor->GetRequestedOutput(), 0.0);
  now += 1ms;
  StepScheduler();
  EXPECT_FALSE(command.IsScheduled());
  EXPECT_DOUBLE_EQ(motor->GetRequestedOutput(), 0.0);
}

TEST_F(WatchedOutputTest, InvalidMotorRequestsStayNeutralAndCannotFeedWatchdog) {
  ASSERT_TRUE(watchdog.RecordCommand(1, true));
  auto command = motor->RunWatchedOutput(
      [] { return std::numeric_limits<double>::quiet_NaN(); }, watchdog);
  command.Schedule();
  Tick();
  EXPECT_DOUBLE_EQ(motor->GetRequestedOutput(), 0.0);
  now += 250ms;
  Tick();
  EXPECT_FALSE(watchdog.IsActive());
  EXPECT_FALSE(command.IsScheduled());
}

TEST_F(WatchedOutputTest, DisableAndStopAllRequireANewUpdateBeforeRestart) {
  ASSERT_TRUE(watchdog.RecordCommand(1, true));
  auto command = motor->RunWatchedOutput([] { return 0.2; }, watchdog);
  command.Schedule();
  Tick();
  ASSERT_DOUBLE_EQ(motor->GetRequestedOutput(), 0.2);
  SetEnabled(false);
  container.StopAll();  // The existing Robot disabled/mode-entry stop path.
  Tick();
  EXPECT_FALSE(command.IsScheduled());
  EXPECT_DOUBLE_EQ(motor->GetRequestedOutput(), 0.0);
  EXPECT_FALSE(watchdog.IsActive());
  SetEnabled(true);
  EXPECT_FALSE(watchdog.RecordCommand(1, true));
  command.Schedule();
  Tick();
  EXPECT_DOUBLE_EQ(motor->GetRequestedOutput(), 0.0);
  ASSERT_TRUE(watchdog.RecordCommand(2, true));
  command.Schedule();
  Tick();
  EXPECT_DOUBLE_EQ(motor->GetRequestedOutput(), 0.2);
}

TEST_F(WatchedOutputTest, FreshnessCannotOverridePhoenixFaultLockout) {
  ASSERT_TRUE(watchdog.RecordCommand(1, true));
  auto command = motor->RunWatchedOutput([] { return 0.2; }, watchdog);
  command.Schedule();
  Tick();
  ASSERT_DOUBLE_EQ(motor->GetRequestedOutput(), 0.2);
  TalonFXSubsystemTestPeer::NeutralResult(*motor,
                                        ctre::phoenix::StatusCode::TxFailed);
  ASSERT_TRUE(watchdog.RecordCommand(2, true));
  Tick();
  EXPECT_TRUE(watchdog.IsActive());
  EXPECT_FALSE(motor->IsReady());
  EXPECT_DOUBLE_EQ(motor->GetRequestedOutput(), 0.0);
  SetEnabled(false);
  container.StopAll();
  Tick();
  EXPECT_TRUE(motor->IsReady());
  EXPECT_FALSE(command.IsScheduled());
  SetEnabled(true);
  Tick();
  EXPECT_DOUBLE_EQ(motor->GetRequestedOutput(), 0.0);
  EXPECT_FALSE(watchdog.IsActive());
}

TEST_F(WatchedOutputTest, FreshZeroRequestRemainsNeutral) {
  ASSERT_TRUE(watchdog.RecordCommand(1, true));
  auto command = motor->RunWatchedOutput([] { return 0.0; }, watchdog);
  command.Schedule();
  Tick();
  EXPECT_TRUE(watchdog.IsActive());
  EXPECT_DOUBLE_EQ(motor->GetRequestedOutput(), 0.0);
  command.Cancel();
  EXPECT_DOUBLE_EQ(motor->GetRequestedOutput(), 0.0);
}

TEST_F(WatchedOutputTest, InterruptionStopsOutputWithoutExtendingFreshness) {
  ASSERT_TRUE(watchdog.RecordCommand(1, true));
  auto command = motor->RunWatchedOutput([] { return 0.2; }, watchdog);
  auto replacement = motor->RunOutput(0.0);
  command.Schedule();
  Tick();
  ASSERT_DOUBLE_EQ(motor->GetRequestedOutput(), 0.2);
  now += 200ms;
  replacement.Schedule();  // Same subsystem requirement interrupts the guard.
  EXPECT_FALSE(command.IsScheduled());
  EXPECT_DOUBLE_EQ(motor->GetRequestedOutput(), 0.0);
  now += 50ms;
  Tick();
  EXPECT_FALSE(watchdog.IsActive());
  EXPECT_DOUBLE_EQ(motor->GetRequestedOutput(), 0.0);
}

TEST_F(WatchedOutputTest, ScheduledCommandFollowsChangingDemandsAndLatestDeadline) {
  ASSERT_TRUE(AcceptDemand(1, 0.2));
  auto command = motor->RunWatchedOutput(
      [this] { return acceptedDemand; }, watchdog);
  command.Schedule();
  StepScheduler();
  ASSERT_DOUBLE_EQ(motor->GetRequestedOutput(), 0.2);

  std::uint64_t updateId = 1;
  for (auto [demand, expected] :
       {std::pair{-0.15, -0.15}, {-1.0, -0.25}, {1.0, 0.4}, {0.1, 0.1}}) {
    now += 100ms;
    ASSERT_TRUE(AcceptDemand(++updateId, demand));
    StepScheduler();
    EXPECT_TRUE(command.IsScheduled());
    EXPECT_DOUBLE_EQ(motor->GetRequestedOutput(), expected);
  }
  now += 250ms - 1ns;
  StepScheduler();
  EXPECT_TRUE(command.IsScheduled());
  EXPECT_DOUBLE_EQ(motor->GetRequestedOutput(), 0.1);
  now += 1ns;
  StepScheduler();
  EXPECT_FALSE(command.IsScheduled());
  EXPECT_DOUBLE_EQ(motor->GetRequestedOutput(), 0.0);
}

TEST_F(WatchedOutputTest, RejectedDemandsCannotReplaceOrExtendAcceptedDemand) {
  ASSERT_TRUE(AcceptDemand(10, 0.2));
  auto command = motor->RunWatchedOutput(
      [this] { return acceptedDemand; }, watchdog);
  command.Schedule();
  StepScheduler();
  ASSERT_DOUBLE_EQ(motor->GetRequestedOutput(), 0.2);
  now += 200ms;
  EXPECT_FALSE(AcceptDemand(10, -0.2));  // Duplicate update.
  EXPECT_FALSE(AcceptDemand(9, 0.0));    // Older update.
  EXPECT_FALSE(AcceptDemand(11, std::numeric_limits<double>::quiet_NaN()));
  EXPECT_FALSE(AcceptDemand(12, -0.2, false));
  StepScheduler();
  EXPECT_DOUBLE_EQ(motor->GetRequestedOutput(), 0.2);
  now += 50ms;
  StepScheduler();
  EXPECT_DOUBLE_EQ(motor->GetRequestedOutput(), 0.0);
  EXPECT_FALSE(command.IsScheduled());

  ASSERT_TRUE(AcceptDemand(13, 0.0));  // Recovery must use this new stop demand.
  StepScheduler();
  EXPECT_FALSE(command.IsScheduled());
  EXPECT_DOUBLE_EQ(motor->GetRequestedOutput(), 0.0);
  command.Schedule();
  StepScheduler();
  EXPECT_TRUE(command.IsScheduled());
  EXPECT_DOUBLE_EQ(motor->GetRequestedOutput(), 0.0);
}

TEST_F(WatchedOutputTest, InvalidSupplierValuesStopMotionWithoutRefreshingTimer) {
  double sampledDemand = 0.2;
  ASSERT_TRUE(watchdog.RecordCommand(1, true));
  auto command = motor->RunWatchedOutput(
      [&sampledDemand] { return sampledDemand; }, watchdog);
  command.Schedule();
  StepScheduler();
  ASSERT_DOUBLE_EQ(motor->GetRequestedOutput(), 0.2);
  std::uint64_t updateId = 1;
  for (double invalid : {std::numeric_limits<double>::quiet_NaN(),
                         std::numeric_limits<double>::infinity(),
                         -std::numeric_limits<double>::infinity()}) {
    now += 60ms;
    sampledDemand = invalid;  // Defensive motor validation still applies.
    EXPECT_FALSE(watchdog.RecordCommand(++updateId, false));
    StepScheduler();
    EXPECT_TRUE(command.IsScheduled());
    EXPECT_DOUBLE_EQ(motor->GetRequestedOutput(), 0.0);
  }
  now += 70ms;
  StepScheduler();
  EXPECT_FALSE(watchdog.IsActive());
  EXPECT_FALSE(command.IsScheduled());
  EXPECT_DOUBLE_EQ(motor->GetRequestedOutput(), 0.0);
}

TEST_F(WatchedOutputTest, InactiveOrExpiredWatchdogDoesNotSampleDemand) {
  int samples = 0;
  auto command = motor->RunWatchedOutput(
      [&samples] {
        ++samples;
        return 0.2;
      },
      watchdog);
  command.Schedule();
  StepScheduler();
  EXPECT_EQ(samples, 0);
  EXPECT_FALSE(command.IsScheduled());
  ASSERT_TRUE(watchdog.RecordCommand(1, true));
  command.Schedule();
  StepScheduler();
  ASSERT_EQ(samples, 1);
  ASSERT_DOUBLE_EQ(motor->GetRequestedOutput(), 0.2);
  now += 250ms;
  StepScheduler();
  EXPECT_EQ(samples, 1);
  EXPECT_FALSE(command.IsScheduled());
  EXPECT_DOUBLE_EQ(motor->GetRequestedOutput(), 0.0);
}

TEST_F(WatchedOutputTest, EmptyDemandSupplierIsRejected) {
  EXPECT_THROW(motor->RunWatchedOutput({}, watchdog), std::invalid_argument);
  EXPECT_FALSE(watchdog.IsActive());
  EXPECT_DOUBLE_EQ(motor->GetRequestedOutput(), 0.0);
}

TEST(MotorConfigTest, RejectsInvalidAddressesAndLimits) {
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const double inf = std::numeric_limits<double>::infinity();
  for (const auto& config :
       {MotorConfig{}, MotorConfig{-1}, MotorConfig{63}, MotorConfig{0, ""},
        MotorConfig{0, "rio", nan, 0.4}, MotorConfig{0, "rio", -0.2, nan},
        MotorConfig{0, "rio", -inf, 0.4}, MotorConfig{0, "rio", -0.2, inf},
        MotorConfig{0, "rio", -1.01, 0.4}, MotorConfig{0, "rio", -0.2, 1.01},
        MotorConfig{0, "rio", 0.1, 0.4}, MotorConfig{0, "rio", -0.4, -0.1},
        MotorConfig{0, "rio", 0.4, -0.4}}) {
    EXPECT_THROW(TalonFXSubsystem::ValidateConfig(config),
                 std::invalid_argument);
    EXPECT_THROW(TalonFXSubsystem{config}, std::invalid_argument);
  }
}

TEST(MotorConfigTest, AcceptsEndpointsAndZeroInclusiveLimits) {
  for (const auto& config :
       {MotorConfig{0, "rio", -1, 1}, MotorConfig{62, "example", 0, 0.2},
        MotorConfig{2, "rio", -0.2, 0}, MotorConfig{3}}) {
    EXPECT_NO_THROW(TalonFXSubsystem::ValidateConfig(config));
  }
}

TEST(MotorConfigTest, DefaultLimitsProhibitMotion) {
  frc::sim::DriverStationSim::SetDsAttached(true);
  frc::sim::DriverStationSim::SetEnabled(true);
  frc::sim::DriverStationSim::NotifyNewData();
  TalonFXSubsystem motor{MotorConfig{40}};
  ASSERT_TRUE(motor.IsReady());
  EXPECT_TRUE(motor.SetOutput(1.0));
  EXPECT_DOUBLE_EQ(motor.GetRequestedOutput(), 0.0);
  EXPECT_TRUE(motor.SetOutput(-1.0));
  EXPECT_DOUBLE_EQ(motor.GetRequestedOutput(), 0.0);
  frc::sim::DriverStationSim::SetEnabled(false);
  frc::sim::DriverStationSim::NotifyNewData();
}
}  // namespace
