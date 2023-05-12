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
};

}  // namespace axes::ecs
