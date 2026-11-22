#pragma once

#include <chrono>
#include <cstdint>
#include <functional>
#include <optional>

// Command freshness policy only: no transport, drivetrain, or motor ownership.
// All calls, including the injected clock, must run on the robot thread.
class DriveCommandWatchdog {
 public:
  using Clock = std::chrono::steady_clock;
  using TimePoint = Clock::time_point;
  using Seconds = std::chrono::duration<double>;
  using Now = std::function<TimePoint()>;
  static constexpr Seconds kDefaultTimeout{0.250};

  // Throws std::invalid_argument for a nonpositive/nonfinite timeout or an
  // empty clock. Production uses a monotonic clock; tests may inject one.
  explicit DriveCommandWatchdog(Seconds timeout = kDefaultTimeout,
                                Now now = Clock::now);

  // The caller validates the whole command. updateId identifies a newly
  // received update, NOT a poll or a change in numerical command values.
  // IDs must strictly increase over this object's lifetime, without wrapping.
  // Invalid updates consume their ID but never refresh the last-valid time.
  bool RecordCommand(std::uint64_t updateId, bool isValid);

  // Evaluates freshness and latches expiration. No command is active at startup
  // or after Reset(); a new valid update is required to become active again.
  bool IsActive();
  void Reset();
  Seconds GetTimeout() const { return m_timeout; }

 private:
  const Seconds m_timeout;
  const Now m_now;
  std::optional<std::uint64_t> m_lastUpdateId;
  std::optional<TimePoint> m_lastValidCommand;
};
