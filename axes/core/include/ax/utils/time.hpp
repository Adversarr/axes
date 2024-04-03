#pragma once
#include "ax/core/entt.hpp"
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

class TimerRegistry {
 public:
  TimerRegistry() = default;
  ~TimerRegistry() {
    for (const auto& [name, duration] : total_durations_) {
      std::cout << "Timer: " << name << " took " << duration << std::endl;
    }
  }

  void AddDuration(const std::string& name, Duration duration) {
    if (total_durations_.find(name) == total_durations_.end()) {
      total_durations_[name] = duration;
    } else {
      total_durations_[name] += duration;
    }
  }

 private:
  std::unordered_map<std::string, Duration> total_durations_;
};

// Use RAII to measure time
class Timer {
public:
  Timer(const std::string& name) : name_(name), start_time_(absl::Now()) {}

  ~Timer() {
    absl::Duration duration = absl::Now() - start_time_;
    if (auto reg = try_get_resource<TimerRegistry>(); reg == nullptr) {
      reg = &add_resource<TimerRegistry>();
      reg->AddDuration(name_, duration);
    } else {
      reg->AddDuration(name_, duration);
    }
  }
private:
  const std::string name_;
  const absl::Time start_time_;
};

#define AX_TIMEIT(name) ::ax::utils::Timer timer_##name(#name)
#ifdef _MSC_VER
#define AX_PRETTY_FUNCTION_NAME __func__
#define AX_TIME_FUNC() ::ax::utils::Timer _timer(__func__)
#else
#define AX_PRETTY_FUNCTION_NAME __PRETTY_FUNCTION__
#define AX_TIME_FUNC() ::ax::utils::Timer _timer(__PRETTY_FUNCTION__)
#endif

}  // namespace ax::utils
