#include "DriveCommandWatchdog.h"

#include <cmath>
#include <stdexcept>
#include <utility>

DriveCommandWatchdog::DriveCommandWatchdog(Seconds timeout, Now now)
    : m_timeout{timeout}, m_now{std::move(now)} {
  if (!std::isfinite(m_timeout.count()) || m_timeout <= Seconds::zero()) {
    throw std::invalid_argument(
        "Drive command timeout must be finite and positive");
  }
  if (!m_now) {
    throw std::invalid_argument("Drive command watchdog requires a clock");
  }
}

bool DriveCommandWatchdog::RecordCommand(std::uint64_t updateId, bool isValid) {
  if (m_lastUpdateId && updateId <= *m_lastUpdateId) {
    return false;
  }
  m_lastUpdateId = updateId;
  if (!isValid) {
    return false;
  }
  m_lastValidCommand = m_now();
  return true;
}

bool DriveCommandWatchdog::IsActive() {
  if (!m_lastValidCommand) {
    return false;
  }
  const auto now = m_now();
  // Fail closed if a test/integration violates the monotonic-clock contract.
  // At the exact deadline the command is already expired.
  if (now < *m_lastValidCommand ||
      Seconds{now - *m_lastValidCommand} >= m_timeout) {
    Reset();
    return false;
  }
  return true;
}

void DriveCommandWatchdog::Reset() {
  m_lastValidCommand.reset();
  // Keep the high-water ID: a stored value must not re-arm after a stop.
}
