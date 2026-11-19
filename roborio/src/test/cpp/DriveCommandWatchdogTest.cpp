#include <gtest/gtest.h>

#include <chrono>
#include <cstdint>
#include <limits>
#include <stdexcept>

#include "DriveCommandWatchdog.h"

namespace {
using namespace std::chrono_literals;
using Watchdog = DriveCommandWatchdog;

class DriveCommandWatchdogTest : public testing::Test {
 protected:
  Watchdog::TimePoint now{10s};
  Watchdog watchdog{250ms, [this] { return now; }};
};

TEST_F(DriveCommandWatchdogTest, StartsInactiveWithoutAnyCommand) {
  EXPECT_FALSE(watchdog.IsActive());
  now += 24h;
  EXPECT_FALSE(watchdog.IsActive());
  watchdog.Reset();
  EXPECT_FALSE(watchdog.IsActive());
}

TEST_F(DriveCommandWatchdogTest, DefaultTimeoutExpiresAt250Milliseconds) {
  Watchdog defaultWatchdog{Watchdog::kDefaultTimeout, [this] { return now; }};
  EXPECT_EQ(defaultWatchdog.GetTimeout(), 250ms);
  ASSERT_TRUE(defaultWatchdog.RecordCommand(0, true));
  now += 250ms - 1ns;
  EXPECT_TRUE(defaultWatchdog.IsActive());
  now += 1ns;
  EXPECT_FALSE(defaultWatchdog.IsActive());
  now += 1ns;
  EXPECT_FALSE(defaultWatchdog.IsActive());
}

TEST_F(DriveCommandWatchdogTest, CustomTimeoutUsesTheConfiguredDeadline) {
  Watchdog custom{125ms, [this] { return now; }};
  ASSERT_TRUE(custom.RecordCommand(1, true));
  now += 125ms - 1ns;
  EXPECT_TRUE(custom.IsActive());
  now += 1ns;
  EXPECT_FALSE(custom.IsActive());
}

TEST(DriveCommandWatchdogConfigTest, RejectsInvalidTimeoutsAndEmptyClock) {
  for (double timeout :
       {0.0, -0.0, -0.1, std::numeric_limits<double>::quiet_NaN(),
        std::numeric_limits<double>::infinity(),
        -std::numeric_limits<double>::infinity()}) {
    EXPECT_THROW((Watchdog{Watchdog::Seconds{timeout}}), std::invalid_argument);
  }
  EXPECT_THROW((Watchdog{250ms, Watchdog::Now{}}), std::invalid_argument);
  EXPECT_NO_THROW((Watchdog{}));
  EXPECT_NO_THROW((Watchdog{1ns}));
  EXPECT_NO_THROW((Watchdog{10s}));
}

TEST_F(DriveCommandWatchdogTest, NewValidUpdateRefreshesTheTimer) {
  ASSERT_TRUE(watchdog.RecordCommand(1, true));
  EXPECT_TRUE(watchdog.IsActive());
  now += 200ms;
  ASSERT_TRUE(watchdog.RecordCommand(2, true));
  now += 250ms - 1ns;
  EXPECT_TRUE(watchdog.IsActive());
  now += 1ns;
  EXPECT_FALSE(watchdog.IsActive());
}

TEST_F(DriveCommandWatchdogTest, DuplicateAndOutOfOrderUpdatesNeverRefresh) {
  ASSERT_TRUE(watchdog.RecordCommand(10, true));
  for (int i = 0; i < 5; ++i) {
    now += 50ms;
    EXPECT_FALSE(watchdog.RecordCommand(10, true));
    EXPECT_FALSE(watchdog.RecordCommand(9, true));
  }
  EXPECT_FALSE(watchdog.IsActive());
  EXPECT_FALSE(watchdog.RecordCommand(10, true));
  EXPECT_FALSE(watchdog.IsActive());
}

TEST_F(DriveCommandWatchdogTest, InvalidUpdatesDoNotArmOrRefresh) {
  EXPECT_FALSE(watchdog.RecordCommand(1, false));
  EXPECT_FALSE(watchdog.IsActive());
  // Re-reading/correcting the same invalid update is still not a new update.
  EXPECT_FALSE(watchdog.RecordCommand(1, true));
  ASSERT_TRUE(watchdog.RecordCommand(2, true));
  now += 200ms;
  EXPECT_FALSE(watchdog.RecordCommand(3, false));
  EXPECT_TRUE(watchdog.IsActive());
  now += 50ms;
  EXPECT_FALSE(watchdog.IsActive());
  EXPECT_FALSE(watchdog.RecordCommand(3, true));
  ASSERT_TRUE(watchdog.RecordCommand(4, true));
  EXPECT_TRUE(watchdog.IsActive());
}

TEST_F(DriveCommandWatchdogTest, ResetRequiresANewerUpdate) {
  ASSERT_TRUE(watchdog.RecordCommand(42, true));
  watchdog.Reset();
  watchdog.Reset();
  EXPECT_FALSE(watchdog.IsActive());
  EXPECT_FALSE(watchdog.RecordCommand(41, true));
  EXPECT_FALSE(watchdog.RecordCommand(42, true));
  EXPECT_FALSE(watchdog.IsActive());
  ASSERT_TRUE(watchdog.RecordCommand(43, true));
  EXPECT_TRUE(watchdog.IsActive());
}

TEST_F(DriveCommandWatchdogTest, NewValidUpdateRecoversAfterLongSilence) {
  ASSERT_TRUE(watchdog.RecordCommand(1, true));
  now += 24h;
  EXPECT_FALSE(watchdog.IsActive());
  ASSERT_TRUE(watchdog.RecordCommand(2, true));
  EXPECT_TRUE(watchdog.IsActive());
  now += 250ms;
  EXPECT_FALSE(watchdog.IsActive());
}

TEST_F(DriveCommandWatchdogTest, NewUpdateCanArriveBeforeExpirationIsPolled) {
  ASSERT_TRUE(watchdog.RecordCommand(1, true));
  now += 1s;
  ASSERT_TRUE(watchdog.RecordCommand(2, true));
  EXPECT_TRUE(watchdog.IsActive());
  now += 250ms;
  EXPECT_FALSE(watchdog.IsActive());
}

TEST_F(DriveCommandWatchdogTest, DistinctUpdatesAtTheSameClockTimeAreAllowed) {
  ASSERT_TRUE(watchdog.RecordCommand(0, true));
  ASSERT_TRUE(watchdog.RecordCommand(1, true));
  EXPECT_TRUE(watchdog.IsActive());
  now += 250ms;
  EXPECT_FALSE(watchdog.IsActive());
}

TEST_F(DriveCommandWatchdogTest, UpdateIdsDoNotWrapAndCannotBeReplayed) {
  constexpr auto maxId = std::numeric_limits<std::uint64_t>::max();
  ASSERT_TRUE(watchdog.RecordCommand(maxId, true));
  now += 250ms;
  EXPECT_FALSE(watchdog.IsActive());
  EXPECT_FALSE(watchdog.RecordCommand(0, true));
  EXPECT_FALSE(watchdog.RecordCommand(maxId, true));
  EXPECT_FALSE(watchdog.IsActive());
}

TEST_F(DriveCommandWatchdogTest, BrokenClockFailsClosedAndExpirationLatches) {
  ASSERT_TRUE(watchdog.RecordCommand(1, true));
  now -= 1ns;
  EXPECT_FALSE(watchdog.IsActive());
  now += 1ns;
  EXPECT_FALSE(watchdog.IsActive());
  ASSERT_TRUE(watchdog.RecordCommand(2, true));
  EXPECT_TRUE(watchdog.IsActive());
}
}  // namespace
