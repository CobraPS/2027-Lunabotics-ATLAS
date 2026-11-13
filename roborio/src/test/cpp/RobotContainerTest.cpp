#include <gtest/gtest.h>

#include <array>
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
