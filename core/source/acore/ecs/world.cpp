#include "acore/ecs/details/world.hpp"

namespace axes::ecs {

EntityID World::counter_ = 0;

std::set<EntityID> World::entities_{};

std::vector<ComponentManagerInfo> World::registered_components_{};

EntityID World::CreateEntity() {
  EntityID id = counter_;
  counter_++;
  return id;
}

void World::DestroyEntity(EntityID entity) {
  if (entities_.contains(entity)) {
    entities_.erase(entity);
    for (const auto& man : registered_components_) {
      man.entity_remover_(entity);
    }
  }
}

bool World::HasEntity(EntityID ent) { return entities_.contains(ent); }

std::vector<std::pair<std::type_index, void*>> World::GetEntityComponents(
    EntityID ent) {
  std::vector<std::pair<std::type_index, void*>> result;
  for (const auto& man : registered_components_) {
    void* ptr = static_cast<void*>(man.querier_(ent));
    if (ptr) {
      result.push_back({man.ti_, ptr});
    }
  }
  return result;
}

std::vector<std::type_index> World::GetRegisteredComponents() {
  std::vector<std::type_index> result;
  result.reserve(registered_components_.size());
  for (const auto & man: registered_components_) {
    result.push_back(man.ti_);
  }
  return result;
}

}  // namespace axes::ecs
