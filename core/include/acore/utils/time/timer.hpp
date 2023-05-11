#pragma once
// TODO: Timer
#include <vector>

#include "time.hpp"

namespace axes::utils {

class MicroSecondTimer {
public:
  void Tick();

  void Tock();

private:
  std::vector<Int64> us_time_stamps_;

  Int64 last_tick_;
};
}  // namespace axes::utils
