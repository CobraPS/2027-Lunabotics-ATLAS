#pragma once

#include <memory>
#include <span>
#include <vector>

#include "Constants.h"
#include "subsystems/TalonFXSubsystem.h"

class RobotContainer {
 public:
  explicit RobotContainer(
      std::span<const MotorConfig> configs = constants::kMotors);
  void StopAll();

 private:
  std::vector<std::unique_ptr<TalonFXSubsystem>> m_motors;
};
