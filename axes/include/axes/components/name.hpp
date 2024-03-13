#pragma once

#include "axes/core/entt.hpp"
namespace ax::cmpt {

struct Name {
  std::string value_;
};

inline Entity create_named_entity(const std::string& name) {
  auto entity = create_entity();
  add_component<Name>(entity, name);
  return entity;
}

inline std::optional<std::string> get_entity_name(Entity entity) {
  if (auto* name = try_get_component<Name>(entity)) {
    return name->value_;
  }
  return std::nullopt;
}

}