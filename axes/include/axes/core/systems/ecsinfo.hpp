#pragma once
#include "axes/core/ecs/ecs.hpp"
namespace axes {

class EcsInfoSystem: public ecs::SystemBase {
public:
  void TickLogic() final;
};

}