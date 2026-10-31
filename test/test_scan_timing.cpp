#include <gtest/gtest.h>

#include "../include/scan_timing.hpp"

using lakibeam1::compute_scan_timing;

namespace
{
rclcpp::Time t(double seconds)
{
  return rclcpp::Time(static_cast<int64_t>(seconds * 1e9), RCL_SYSTEM_TIME);
}
}  // namespace

TEST(ScanTiming, StampIsRevolutionStartNotNextRevolution)
{
  // Revolution began at 100.000 s; the next one's first packet arrived 33.3 ms later.
  const auto timing = compute_scan_timing(t(100.000), t(100.0333), 1200, 0.0);
  ASSERT_TRUE(timing.valid);
  EXPECT_EQ(timing.stamp.nanoseconds(), t(100.000).nanoseconds());
}

TEST(ScanTiming, TimeOffsetShiftsStamp)
{
  const auto timing = compute_scan_timing(t(100.000), t(100.0333), 1200, -0.017);
  ASSERT_TRUE(timing.valid);
  EXPECT_NEAR(timing.stamp.seconds(), 99.983, 1e-6);
}

TEST(ScanTiming, ScanTimeAndTimeIncrementCoverTheWholeRevolution)
{
  const auto timing = compute_scan_timing(t(100.000), t(100.0333), 1200, 0.0);
  ASSERT_TRUE(timing.valid);
  EXPECT_NEAR(timing.scan_time, 0.0333, 1e-6);
  // One beam period, so num_readings * time_increment == scan_time (REP-117).
  EXPECT_NEAR(timing.time_increment * 1200, timing.scan_time, 1e-6);
}

TEST(ScanTiming, FirstPartialRevolutionIsInvalid)
{
  // At startup the revolution start is unknown (default-constructed time).
  const auto timing = compute_scan_timing(rclcpp::Time(0, 0, RCL_SYSTEM_TIME), t(100.0), 700, 0.0);
  EXPECT_FALSE(timing.valid);
}

TEST(ScanTiming, NonIncreasingArrivalTimesAreInvalid)
{
  EXPECT_FALSE(compute_scan_timing(t(100.0), t(100.0), 1200, 0.0).valid);
  EXPECT_FALSE(compute_scan_timing(t(100.1), t(100.0), 1200, 0.0).valid);
}

TEST(ScanTiming, EmptyRevolutionIsInvalid)
{
  EXPECT_FALSE(compute_scan_timing(t(100.0), t(100.0333), 0, 0.0).valid);
}
