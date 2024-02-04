#pragma once
#include <absl/time/clock.h>
#include <absl/time/time.h>

namespace ax::utils {

/****************************** Classes ******************************/
using absl::Duration;
using absl::Time;

using absl::Microseconds;
using absl::Milliseconds;
using absl::Nanoseconds;
using absl::Seconds;

/****************************** Methods ******************************/
using absl::GetCurrentTimeNanos;  // void -> int64
using absl::Now;                  // void -> Time
using absl::SleepFor;             // Duration -> void
}  // namespace ax::utils
