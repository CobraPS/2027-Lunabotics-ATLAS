#include "subsystems/TalonFXSubsystem.h"

#include <frc/DriverStation.h>
#include <frc/Errors.h>
#include <frc/RobotController.h>

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <utility>

MotorConfig TalonFXSubsystem::ValidateConfig(MotorConfig config) {
  if (config.canId < 0 || config.canId > 62 || config.canBus.empty()) {
    throw std::invalid_argument("Motor CAN ID must be 0..62 and bus nonempty");
  }
  if (!std::isfinite(config.minOutput) || !std::isfinite(config.maxOutput) ||
      config.minOutput < -1.0 || config.maxOutput > 1.0 ||
      config.minOutput > 0.0 || config.maxOutput < 0.0 ||
      config.minOutput > config.maxOutput) {
    throw std::invalid_argument(
        "Motor output bounds must be finite, within [-1, 1], and include zero");
  }
  return config;
}

TalonFXSubsystem::TalonFXSubsystem(MotorConfig config)
    : m_config{ValidateConfig(std::move(config))},
      m_motor{m_config.canId, ctre::phoenix6::CANBus{m_config.canBus}} {
  Stop();
  // Apply deterministic defaults as well as a second layer of output bounds.
  // Mechanism-specific inversion/current limits must be reviewed before use.
  ctre::phoenix6::configs::TalonFXConfiguration deviceConfig{};
  deviceConfig.MotorOutput.PeakForwardDutyCycle = m_config.maxOutput;
  deviceConfig.MotorOutput.PeakReverseDutyCycle = m_config.minOutput;
  RecordConfigurationResult(m_motor.GetConfigurator().Apply(deviceConfig));
  m_request.WithEnableFOC(false);  // No Phoenix Pro license required.
  Stop();
  if (!m_configured) {
    FRC_ReportError(
        frc::err::Error,
        "Talon FX configuration failed; motor output locked at zero");
  } else if (!m_ready) {
    FRC_ReportError(
        frc::err::Error,
        "Talon FX neutral request failed; disable and retry before motion");
  }
}

TalonFXSubsystem::~TalonFXSubsystem() { Stop(); }

bool TalonFXSubsystem::SetOutput(double dutyCycle) {
  if (!m_ready || !frc::DriverStation::IsEnabled() ||
      !std::isfinite(dutyCycle)) {
    Stop();
    return false;
  }
  const double output =
      std::clamp(dutyCycle, m_config.minOutput, m_config.maxOutput);
  if (output == 0.0) {
    Stop();
    return m_ready;
  }
  if (!m_motor.SetControl(m_request.WithOutput(output)).IsOK()) {
    m_ready = false;  // Require disabled-state neutral recovery before motion.
    Stop();
    FRC_ReportError(
        frc::err::Error,
        "Talon FX control failed; disable and confirm neutral before retrying");
    return false;
  }
  m_requestedOutput = output;
  return true;
}

void TalonFXSubsystem::Stop() {
  RecordNeutralResult(
      m_motor.SetControl(ctre::phoenix6::controls::NeutralOut{}));
}

void TalonFXSubsystem::RecordConfigurationResult(
    ctre::phoenix::StatusCode status) {
  m_configured = status.IsOK();
  m_ready = m_configured;
}

void TalonFXSubsystem::RecordNeutralResult(ctre::phoenix::StatusCode status) {
  m_requestedOutput = 0.0;
  m_request.WithOutput(0.0);
  if (!status.IsOK()) {
    m_ready = false;
  } else if (!frc::DriverStation::IsEnabled()) {
    // A transient control failure must not require reconstruction, but never
    // re-arm while enabled or use a neutral success to mask failed setup.
    m_ready = m_configured;
  }
}

void TalonFXSubsystem::Periodic() {
  // Clear disabled requests and retry neutral after a control fault, including
  // while enabled. A successful retry while enabled must not re-arm motion.
  if (!frc::DriverStation::IsEnabled() || !m_ready) {
    Stop();
  }
}

void TalonFXSubsystem::SimulationPeriodic() {
  m_motor.GetSimState().SetSupplyVoltage(
      frc::RobotController::GetBatteryVoltage());
}

frc2::CommandPtr TalonFXSubsystem::RunOutput(double dutyCycle) {
  return RunEnd([this, dutyCycle] { SetOutput(dutyCycle); },
                [this] { Stop(); });
}

double TalonFXSubsystem::GetMeasuredOutput() {
  return m_motor.GetDutyCycle().GetValueAsDouble();
}
