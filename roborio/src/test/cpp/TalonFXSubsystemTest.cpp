#include <frc/DriverStation.h>
#include <frc/simulation/DriverStationSim.h>
#include <frc2/command/CommandScheduler.h>
#include <gtest/gtest.h>

#include <chrono>
#include <limits>
#include <memory>
#include <thread>

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
