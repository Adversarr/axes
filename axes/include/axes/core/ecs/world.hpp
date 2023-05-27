#pragma once
#include <absl/container/flat_hash_map.h>
#include <absl/container/flat_hash_set.h>

#include <iostream>
#include <map>
#include <memory>
#include <optional>
#include <set>
#include <typeindex>

#include "axes/core/ecs/common.hpp"
#include "axes/core/ecs/event.hpp"
#include "axes/core/ecs/systems.hpp"

namespace axes::ecs {

/**
 * @class World
 * @brief World, manages all the entities, and methods that does not do with
 *  specific Component.
 *
 */
class World {
public:
  /**
   * @brief Creates a new entity
   *
   * @return
   */
  static EntityID CreateEntity();

  /**
   * @brief Removes an entity
   *
   * @param entity
   */
  static void DestroyEntity(EntityID entity);

  /**
   * @brief Return true if the world has given entity.
   *
   * @param ent
   * @return
   */
  static bool HasEntity(EntityID ent);

  /**
   * @brief Returns the component type & ptr corresponding to ent.
   *
   * @param ent
   */
  static std::vector<std::pair<std::type_index, void*>> GetEntityComponents(
      EntityID ent);

  /**
   * Register a component in the world,
   *
   * @warning this method should not be called from user.
   * @tparam Component
   * @param info
   */
  static void RegisterComponent(ComponentManagerInfo info);

  /**
   * get all the registered components for reflection.
   * @return registered components.
   */
  static const std::vector<ComponentManagerInfo>& GetRegisteredComponents();

  /**
   * @brief Try to register a system
   *
   * @param system
   */
  static void TryRegisterSystem(std::shared_ptr<SystemBase> system);

  /**
   * @brief Try to destroy a system
   *
   * @param system
   */
  static void TryDestroySystem(SystemBase* system);

  /**
   * @brief Startup Main Loop
   *
   * @return
   */
  static int MainLoop(bool shutdown_axes = false);

  static void DestroyAll();

  static void EnqueueEvent(Event evt);

  static const absl::flat_hash_set<EntityID>& GetEntities();

  struct RunningSystemInfo {
    std::shared_ptr<SystemBase> system_;
    bool enable_;
    bool operator<(const RunningSystemInfo&) const noexcept;
  };

private:
  static void PreLoop();
  static void LoopBody();
  static void PostLoop();
};

}  // namespace axes::ecs
