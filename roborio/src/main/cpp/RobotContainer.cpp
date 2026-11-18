#include "RobotContainer.h"

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
