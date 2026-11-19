#pragma once

#include <memory>
#include <span>
#include <vector>

#include "Constants.h"
#include "DriveCommandWatchdog.h"
#include "subsystems/TalonFXSubsystem.h"

class RobotContainer {
 public:
  explicit RobotContainer(
      std::span<const MotorConfig> configs = constants::kMotors,
      DriveCommandWatchdog::Seconds driveCommandTimeout =
          DriveCommandWatchdog::kDefaultTimeout,
      DriveCommandWatchdog::Now now = DriveCommandWatchdog::Clock::now);
  void StopAll();
  // Future receiver/mode integration must record actual new validated updates
  // here and use this gate for drive outputs. No drive roles are assigned yet.
  DriveCommandWatchdog& GetDriveCommandWatchdog() { return m_driveWatchdog; }

 private:
  DriveCommandWatchdog m_driveWatchdog;
  std::vector<std::unique_ptr<TalonFXSubsystem>> m_motors;
};
