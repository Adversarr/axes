#pragma once

#include <span>
#include <vector>

#include "common.hpp"
#include "componentmanager.hpp"

namespace axes::ecs {

class EntityManager {
public:
  explicit EntityManager(EntityID id) : id_(id) {}

  EntityManager(const EntityManager&) = default;

  EntityManager(EntityManager&&) = default;

  template <typename Component, typename... Args>
  Component* Attach(Args&&... args) const;

  template <typename Component, typename... Args>
  const EntityManager& Replace(Args&&... args) const;

  template <typename Component> const EntityManager& Detach() const;

  void Destroy();

  EntityID GetEntityID() const noexcept { return id_; }

private:
  const EntityID id_;
};

template <typename Component>
const EntityManager& EntityManager::Detach() const {
  ComponentManager<Component>{}.DetachComponent(id_);
  return *this;
}

template <typename Component, typename... Args>
const EntityManager& EntityManager::Replace(Args&&... args) const {
  ComponentManager<Component>{}.ReplaceComponent(id_,
                                                 std::forward<Args>(args)...);
  return *this;
}

template <typename Component, typename... Args>
Component* EntityManager::Attach(Args&&... args) const {
  return ComponentManager<Component>{}.AttachComponent(id_,
                                                std::forward<Args>(args)...);
}

}  // namespace axes::ecs
