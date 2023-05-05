#pragma once

#include "common.hpp"
#include <span>

namespace axes {

class EntityManager {
public:
  EntityID CreateEntity();
  void DestroyEntity(EntityID id);
};

}