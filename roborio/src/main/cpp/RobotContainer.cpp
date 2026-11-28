#include "RobotContainer.h"
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/RobotBase.h>

#include <set>
#include <stdexcept>
#include <string>
#include <utility>

RobotContainer::RobotContainer(std::span<const MotorConfig> configs) {
  // Validate the whole collection before constructing any CAN devices.
  std::set<std::pair<std::string, int>> addresses;
  std::set<int> simulationIds;
  for (const auto& config : configs) {
    TalonFXSubsystem::ValidateConfig(config);
    if (!addresses.emplace(config.canBus, config.canId).second) {
      throw std::invalid_argument("Duplicate motor CAN address");
    }
    // Phoenix software simulation puts every device on the same virtual bus.
    if (frc::RobotBase::IsSimulation() &&
        !simulationIds.insert(config.canId).second) {
      throw std::invalid_argument(
          "Simulation requires globally unique CAN IDs");
    }
  }
  for (const auto& config : configs) {
    m_motors.push_back(std::make_unique<TalonFXSubsystem>(config));
  }
  // No input bindings, default commands, autonomous motion, or NT receiver.
}

void RobotContainer::StopAll() {
  for (const auto& motor : m_motors) {
    motor->Stop();
  }
}

void RobotContainer::Periodic() {
  m_driveCommand = m_driveReceiver.Read();
  // Publish what the RoboRIO received so it can be checked in a dashboard.
  frc::SmartDashboard::PutNumber("Received/DriveForward",
                                 m_driveCommand.forward);
  frc::SmartDashboard::PutNumber("Received/DriveTurn", m_driveCommand.turn);
  frc::SmartDashboard::PutBoolean("Received/DriveValid", m_driveCommand.valid);
}
