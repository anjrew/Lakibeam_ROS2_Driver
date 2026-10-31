#pragma once

#include <cstddef>

#include <rclcpp/duration.hpp>
#include <rclcpp/time.hpp>

namespace lakibeam1
{

struct ScanTiming
{
  rclcpp::Time stamp;
  float scan_time{0.0f};
  float time_increment{0.0f};
  bool valid{false};
};

// Timing for one published revolution.
//
// The driver only learns a revolution is complete when the azimuth-0 packet of
// the NEXT revolution arrives, so it publishes one scan period after the
// revolution began. REP-117 wants header.stamp at the first beam, so stamp with
// the revolution's own start (`revolution_start`), not the arrival of the next
// one; that was ~one scan period (33 ms at 30 Hz) late.
//
// `time_offset_s` is added to the stamp to cancel the sensor's internal output
// latency (packets leave the sensor after the beams were measured). Negative
// moves the stamp earlier. Measure it per sensor, e.g. by cross-correlating the
// scan-matched yaw rate against a gyro.
//
// Invalid when the revolution start is unknown (the first, partial revolution
// after startup) or the two arrival times are not increasing.
inline ScanTiming compute_scan_timing(
  const rclcpp::Time & revolution_start, const rclcpp::Time & next_revolution_start,
  std::size_t num_readings, double time_offset_s)
{
  ScanTiming t;
  if (revolution_start.nanoseconds() == 0 || num_readings == 0 ||
    next_revolution_start <= revolution_start)
  {
    return t;
  }
  const double duration = (next_revolution_start - revolution_start).seconds();
  t.stamp = revolution_start + rclcpp::Duration::from_seconds(time_offset_s);
  t.scan_time = static_cast<float>(duration);
  t.time_increment = static_cast<float>(duration / static_cast<double>(num_readings));
  t.valid = true;
  return t;
}

}  // namespace lakibeam1
