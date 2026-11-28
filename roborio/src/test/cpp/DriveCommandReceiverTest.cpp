#include <gtest/gtest.h>

#include <limits>

#include "DriveCommandReceiver.h"

class DriveCommandReceiverTest : public ::testing::Test {
 protected:
  void SetUp() override { m_inst = nt::NetworkTableInstance::Create(); }
  void TearDown() override { nt::NetworkTableInstance::Destroy(m_inst); }

  nt::NetworkTableInstance m_inst;
};

TEST_F(DriveCommandReceiverTest, NoDataIsInvalidAndZero) {
  DriveCommandReceiver receiver{m_inst};
  const auto cmd = receiver.Read();
  EXPECT_FALSE(cmd.valid);
  EXPECT_EQ(cmd.forward, 0.0);
  EXPECT_EQ(cmd.turn, 0.0);
}

TEST_F(DriveCommandReceiverTest, ReadsPublishedValues) {
  DriveCommandReceiver receiver{m_inst};
  auto fwd =
      m_inst.GetDoubleTopic("/SmartDashboard/Jetson/DriveForward").Publish();
  auto trn =
      m_inst.GetDoubleTopic("/SmartDashboard/Jetson/DriveTurn").Publish();
  fwd.Set(0.5);
  trn.Set(-0.2);
  const auto cmd = receiver.Read();
  EXPECT_TRUE(cmd.valid);
  EXPECT_DOUBLE_EQ(cmd.forward, 0.5);
  EXPECT_DOUBLE_EQ(cmd.turn, -0.2);
}

TEST_F(DriveCommandReceiverTest, FollowsChangedValues) {
  DriveCommandReceiver receiver{m_inst};
  auto fwd =
      m_inst.GetDoubleTopic("/SmartDashboard/Jetson/DriveForward").Publish();
  auto trn =
      m_inst.GetDoubleTopic("/SmartDashboard/Jetson/DriveTurn").Publish();
  fwd.Set(0.5);
  trn.Set(0.0);
  EXPECT_DOUBLE_EQ(receiver.Read().forward, 0.5);
  fwd.Set(0.1);
  EXPECT_DOUBLE_EQ(receiver.Read().forward, 0.1);
}

TEST_F(DriveCommandReceiverTest, OnlyOneEntryPublishedIsInvalid) {
  DriveCommandReceiver receiver{m_inst};
  auto fwd =
      m_inst.GetDoubleTopic("/SmartDashboard/Jetson/DriveForward").Publish();
  fwd.Set(0.5);
  EXPECT_FALSE(receiver.Read().valid);
}

TEST_F(DriveCommandReceiverTest, RejectsNaNAndInfinity) {
  DriveCommandReceiver receiver{m_inst};
  auto fwd =
      m_inst.GetDoubleTopic("/SmartDashboard/Jetson/DriveForward").Publish();
  auto trn =
      m_inst.GetDoubleTopic("/SmartDashboard/Jetson/DriveTurn").Publish();
  trn.Set(0.0);

  fwd.Set(std::numeric_limits<double>::quiet_NaN());
  EXPECT_FALSE(receiver.Read().valid);

  fwd.Set(std::numeric_limits<double>::infinity());
  const auto cmd = receiver.Read();
  EXPECT_FALSE(cmd.valid);
  EXPECT_EQ(cmd.forward, 0.0);
}
