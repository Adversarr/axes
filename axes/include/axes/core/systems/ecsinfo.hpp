#pragma once
#include "axes/core/ecs/ecs.hpp"
namespace axes {

struct EcsInfoSystemRunningStat {
  bool enable_ = false;
  static void InitResource();
};

class EcsInfoSystem : public ecs::SystemBase {
public:
  void TickLogic() final;
};

}  // namespace axes
