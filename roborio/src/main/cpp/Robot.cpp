#include "Robot.h"

#include <frc2/command/CommandScheduler.h>

void Robot::RobotPeriodic() { frc2::CommandScheduler::GetInstance().Run(); }

void Robot::DisabledInit() {
  frc2::CommandScheduler::GetInstance().CancelAll();
  m_container.StopAll();
}

void Robot::DisabledPeriodic() { m_container.StopAll(); }

void Robot::AutonomousInit() {
  frc2::CommandScheduler::GetInstance().CancelAll();
  m_container.StopAll();
}

void Robot::TeleopInit() {
  frc2::CommandScheduler::GetInstance().CancelAll();
  m_container.StopAll();
}

void Robot::TestInit() {
  frc2::CommandScheduler::GetInstance().CancelAll();
  m_container.StopAll();
}

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif
