#pragma once

#include "axes/core/entt.hpp"
namespace ax::cmpt {

struct Name {
  std::string value_;
};
Entity create_named_entity(const std::string& name);
std::optional<std::string> get_entity_name(Entity entity);


}