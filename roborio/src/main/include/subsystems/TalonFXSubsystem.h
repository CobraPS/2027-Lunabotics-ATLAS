#pragma once

#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>

#include <ctre/phoenix6/TalonFX.hpp>
#include <string>

// Duty-cycle bounds must include zero and lie within [-1, 1]. The default
// limits prohibit motion; the caller must explicitly choose usable limits.
struct MotorConfig {
  int canId = -1;  // Unassigned; validation requires an explicit valid ID.
  std::string canBus = "rio";
  double minOutput = 0.0;
  double maxOutput = 0.0;
};

// One independently configurable motor, not a drivetrain. Call on the robot
// thread; commands using this motor must require this subsystem.
class TalonFXSubsystem : public frc2::SubsystemBase {
 public:
  explicit TalonFXSubsystem(MotorConfig config);
  ~TalonFXSubsystem() override;

  // Throws std::invalid_argument before constructing a device on bad config.
  static MotorConfig ValidateConfig(MotorConfig config);

  // Returns false for disabled/unready/invalid requests or a Phoenix error.
  // Finite requests outside the configured bounds are clamped and accepted.
  bool SetOutput(double dutyCycle);
  void Stop();
  void Periodic() override;
  void SimulationPeriodic() override;

  // Stops on command cancellation, interruption, or normal end.
  frc2::CommandPtr RunOutput(double dutyCycle);

  double GetRequestedOutput() const { return m_requestedOutput; }
  // Configuration must succeed; control faults recover only after a successful
  // neutral request while disabled. Recovery never restores a stored output.
  bool IsReady() const { return m_ready; }

  // Asynchronous device telemetry, distinct from the software request.
  double GetMeasuredOutput();

 private:
  // Keep vendor-result handling testable without simulating physical CAN loss.
  friend class TalonFXSubsystemTestPeer;
  void RecordConfigurationResult(ctre::phoenix::StatusCode status);
  void RecordNeutralResult(ctre::phoenix::StatusCode status);

  const MotorConfig m_config;
  ctre::phoenix6::hardware::TalonFX m_motor;
  ctre::phoenix6::controls::DutyCycleOut m_request{0.0};
  double m_requestedOutput = 0.0;
  bool m_configured = false;
  bool m_ready = false;
};
