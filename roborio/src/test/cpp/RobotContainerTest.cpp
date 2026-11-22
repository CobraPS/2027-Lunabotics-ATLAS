#include <gtest/gtest.h>

#include <array>
#include <chrono>
#include <stdexcept>

#include "RobotContainer.h"

TEST(RobotContainerTest, EmptyHardwareConfigurationIsRunnable) {
  RobotContainer container;
  EXPECT_NO_THROW(container.StopAll());
}

TEST(RobotContainerTest, RejectsDuplicateAddressesBeforeDeviceConstruction) {
  const std::array configs{MotorConfig{12}, MotorConfig{12}};
  EXPECT_THROW(RobotContainer{configs}, std::invalid_argument);
}

TEST(RobotContainerTest, RejectsSharedSimulationIdsAcrossBuses) {
  const std::array configs{MotorConfig{12, "rio"}, MotorConfig{12, "example"}};
  EXPECT_THROW(RobotContainer{configs}, std::invalid_argument);
}

TEST(RobotContainerTest, StopAllDisarmsWatchdogWithoutForgettingUpdateIds) {
  DriveCommandWatchdog::TimePoint now{};
  RobotContainer container{constants::kMotors, std::chrono::milliseconds{100},
                           [&now] { return now; }};
  auto& watchdog = container.GetDriveCommandWatchdog();
  EXPECT_FALSE(watchdog.IsActive());
  EXPECT_EQ(watchdog.GetTimeout(), std::chrono::milliseconds{100});
  ASSERT_TRUE(watchdog.RecordCommand(1, true));
  container.StopAll();  // Existing disable and mode-entry hooks call this.
  EXPECT_FALSE(watchdog.IsActive());
  EXPECT_FALSE(watchdog.RecordCommand(1, true));
  ASSERT_TRUE(watchdog.RecordCommand(2, true));
  EXPECT_TRUE(watchdog.IsActive());
}

TEST(RobotContainerTest, RejectsInvalidWatchdogConfigBeforeHardwareSetup) {
  EXPECT_THROW((RobotContainer{constants::kMotors,
                               DriveCommandWatchdog::Seconds{0}}),
               std::invalid_argument);
}
