#pragma once

#include <memory>
#include <span>
#include <vector>

#include "Constants.h"
#include "subsystems/TalonFXSubsystem.h"
#include "DriveCommandReceiver.h"

class RobotContainer {
 public:
  explicit RobotContainer(
      std::span<const MotorConfig> configs = constants::kMotors);
  void StopAll();
  void Periodic();  // call once per robot cycle
  const DriveCommand& GetDriveCommand() const { return m_driveCommand; }

 private:
  std::vector<std::unique_ptr<TalonFXSubsystem>> m_motors;
  DriveCommandReceiver m_driveReceiver;
  DriveCommand m_driveCommand;
};
