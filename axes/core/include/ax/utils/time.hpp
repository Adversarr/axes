#pragma once
#include <iostream>
#include <thread>
#include <chrono>

#include "ax/core/entt.hpp"

namespace ax::utils {

using std::this_thread::sleep_for;  // Duration -> void
using clock_t = std::chrono::steady_clock;
using duration_t = typename clock_t::duration;
using time_point_t = typename clock_t::time_point;

using nanoseconds = std::chrono::nanoseconds;
using microseconds = std::chrono::microseconds;
using milliseconds = std::chrono::milliseconds;
using seconds = std::chrono::seconds;

namespace details {

inline bool timer_en(bool set, bool val) {
  static bool en = false;
  if (set) {
    en = val;
  }
  return en;
}

}

inline bool is_timer_enabled() {
  return details::timer_en(false, false);
}

inline bool set_timer_enable(bool en) {
  return details::timer_en(true, en);
}

inline time_point_t now() { return clock_t::now(); }

inline int64_t get_current_time_nanos() {
  auto current = std::chrono::high_resolution_clock::now();
  return std::chrono::duration_cast<nanoseconds>(current.time_since_epoch()).count();
}

class TimerRegistry {
public:
  TimerRegistry() = default;
  ~TimerRegistry() {
    for (const auto& [name, desc] : total_durations_) {
      auto total = std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(desc.total_);
      auto cnt = static_cast<double>(desc.cnt_);
      std::cout << "Timer: " << name << "\n- total: " << total << "\n- count: " << desc.cnt_
                << "\n- avg:   " << (total / cnt) << '\n';
    }
  }

  struct Desc {
    duration_t total_;
    Index cnt_ = 0;
  };

  void AddDuration(const char* name, duration_t duration) {
    if (is_timer_enabled()) {
      if (total_durations_.find(name) == total_durations_.end()) {
        auto [it, _] = total_durations_.emplace(name, Desc{});
        it->second.total_ = duration;
        it->second.cnt_ = 1;
      } else {
        auto& cur = total_durations_[name];
        cur.total_ += duration;
        cur.cnt_ += 1;
      }
    }
  }

private:
  std::unordered_map<std::string, Desc> total_durations_;
};

// Use RAII to measure time
class Timer {
public:
  explicit inline Timer(const char* name, bool en = true) : name_(name), en_(en) {
    if (en_) {
      start_time_ = now();
    }
  }

  inline ~Timer() {
    if (en_) {
      duration_t duration = now() - start_time_;
      if (auto* reg = try_get_resource<TimerRegistry>(); reg == nullptr) {
        reg = &add_resource<TimerRegistry>();
        reg->AddDuration(name_, duration);
      } else {
        reg->AddDuration(name_, duration);
      }
    }
  }

private:
  const char* name_;
  time_point_t start_time_;
  const bool en_;
};


#ifdef AX_ENABLE_TIMER
#define AX_TIMEIT(name) ::ax::utils::Timer timer_do_not_use(#name)
#ifdef _MSC_VER
#  define AX_PRETTY_FUNCTION_NAME __func__
#  define AX_TIME_FUNC() ::ax::utils::Timer _timer(__func__)
#else
#  define AX_PRETTY_FUNCTION_NAME __PRETTY_FUNCTION__
#  define AX_TIME_FUNC() ::ax::utils::Timer _timer(__PRETTY_FUNCTION__)
#endif
#else
#define AX_TIMEIT(name) ((void) name)
#define AX_TIME_FUNC() ((void) 0)
#endif

}  // namespace ax::utils
