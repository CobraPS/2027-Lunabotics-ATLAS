#include "DriveCommandReceiver.h"

#include <cmath>

namespace {
// Same names as the ROS 2 bridge (atlas_roborio_bridge).
constexpr const char* kForwardTopic = "/SmartDashboard/Jetson/DriveForward";
constexpr const char* kTurnTopic = "/SmartDashboard/Jetson/DriveTurn";
}  // namespace

DriveCommandReceiver::DriveCommandReceiver(nt::NetworkTableInstance instance)
    : m_forward{instance.GetDoubleTopic(kForwardTopic).Subscribe(0.0)},
      m_turn{instance.GetDoubleTopic(kTurnTopic).Subscribe(0.0)} {}

DriveCommand DriveCommandReceiver::Read() {
  const auto forward = m_forward.GetAtomic();
  const auto turn = m_turn.GetAtomic();

  DriveCommand command;  // zeros, valid = false

  // time == 0 means nothing was ever published on that entry.
  if (forward.time == 0 || turn.time == 0) {
    return command;
  }
  // Same rule as the bridge: reject NaN and infinity.
  if (!std::isfinite(forward.value) || !std::isfinite(turn.value)) {
    return command;
  }

  command.forward = forward.value;
  command.turn = turn.value;
  command.valid = true;
  return command;
}
