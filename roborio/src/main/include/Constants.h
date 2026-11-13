#pragma once

#include <array>

#include "subsystems/TalonFXSubsystem.h"

namespace constants {
// Intentionally no hardware assignments. Add one MotorConfig per confirmed
// motor here after the team reviews CAN IDs, buses, and duty-cycle bounds.
inline const std::array<MotorConfig, 0> kMotors{};
}  // namespace constants
