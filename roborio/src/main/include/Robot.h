#pragma once

#include <frc/TimedRobot.h>

#include "RobotContainer.h"

class Robot : public frc::TimedRobot {
 public:
  void RobotPeriodic() override;
  void DisabledInit() override;
  void DisabledPeriodic() override;
  void AutonomousInit() override;
  void TeleopInit() override;
  void TestInit() override;

 private:
  RobotContainer m_container;
};
