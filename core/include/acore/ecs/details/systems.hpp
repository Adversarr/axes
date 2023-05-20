#pragma once

#include <string>

#include "acore/common.hpp"

namespace axes::ecs {

enum class SystemPriority : UInt32 {
  kLowest = 1,
  kLow = 2,
  kMedium = 3,
  kHigh = 4,
  kHighest = 5,
  kUser = 6,
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

  explicit SystemBase(bool enable_on_init = false,
                      SystemPriority base_priority = SystemPriority::kLowest)
      : enable_on_init_(enable_on_init), base_priority_(base_priority) {}

private:
  bool enable_on_init_;

  SystemPriority base_priority_;
};

}  // namespace axes::ecs
