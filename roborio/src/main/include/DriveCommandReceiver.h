#pragma once

#include <networktables/DoubleTopic.h>
#include <networktables/NetworkTableInstance.h>

// Latest drive command received from the Jetson bridge.
struct DriveCommand {
  double forward = 0.0;  // linear.x from /cmd_vel (raw, not scaled)
  double turn = 0.0;     // angular.z from /cmd_vel (raw, not scaled)
  bool valid = false;    // false if data is missing or not a finite number
};

class DriveCommandReceiver {
 public:
  explicit DriveCommandReceiver(
      nt::NetworkTableInstance instance =
          nt::NetworkTableInstance::GetDefault());

  // Read the latest values. Returns zeros with valid = false when the Jetson
  // has not published both entries or sent NaN/infinity.
  DriveCommand Read();

 private:
  nt::DoubleSubscriber m_forward;
  nt::DoubleSubscriber m_turn;
};
