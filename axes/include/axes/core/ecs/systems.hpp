#pragma once

#include <string>

#include "axes/core/common.hpp"

namespace axes::ecs {

enum class SystemBasePriority : UInt32 {
  kLowest = 1,
  kLow = 2,
  kMedium = 3,
  kHigh = 4,
  kHighest = 5,
};

class SystemBase {
public:
  virtual ~SystemBase() = default;

  virtual void TickLogic();

  virtual void TickRender();

  virtual void Reset();

  virtual void Initialize();

  virtual std::string GetName() const;

  virtual UInt32 GetUserPriority() const { return 0; }

  UInt64 GetPriority() const {
    return GetUserPriority() + (static_cast<UInt64>(base_priority_) << 32);
  }

  explicit SystemBase(bool enable_on_init = false, SystemBasePriority base_priority
                                                   = SystemBasePriority::kLowest)
      : enable_on_init_(enable_on_init), base_priority_(base_priority) {}

private:
  bool enable_on_init_;

  SystemBasePriority base_priority_;
};

}  // namespace axes::ecs
