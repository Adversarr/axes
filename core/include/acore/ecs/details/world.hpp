#pragma once
#include <iostream>
#include <map>
#include <optional>
#include <set>
#include <memory>
#include <typeindex>
#include <unordered_map>

#include "acore/ecs/details/common.hpp"
#include "acore/ecs/details/event.hpp"
#include "acore/ecs/details/systems.hpp"

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

  template <typename OutputIt> static void QueryAllEntities(OutputIt out);

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
  template <typename Component>
  static void RegisterComponent(ComponentManagerInfo info);

  /**
   * get all the registered components for reflection.
   * @return registered components.
   */
  static std::vector<std::type_index> GetRegisteredComponents();

  /**
   * @brief Try to register a system
   *
   * @param system
   */
  static void TryRegisterSystem(SystemBase* system);

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

  static void EnqueueEvent(Event evt);

  struct RegisterSystemInfo {
    // TODO: To register some system ...
    std::unique_ptr<SystemBase> system_;
    bool enable_after_register_{false};
    bool require_destroy_{false};
  };

  struct RunningSystemInfo {
    SystemBase* system_;
    bool enable_;
  };

private:
  static void PreLoop();
  static void LoopBody();
  static void PostLoop();
  static std::set<EntityID> entities_;
  static std::vector<ComponentManagerInfo> registered_components_;
  static std::vector<RunningSystemInfo> systems_running_;
  static std::vector<SystemBase*> systems_waiting_;
  static std::vector<SystemBase*> systems_destroying_;
  static bool is_running_;
  static int return_value_;
  static std::vector<Event> events_;
  static EntityID counter_;
};

template <typename OutputIt> void World::QueryAllEntities(OutputIt out) {
  std::copy(entities_.begin(), entities_.end(), out);
}

template <typename Component>
void World::RegisterComponent(ComponentManagerInfo info) {
  registered_components_.push_back(info);
}
}  // namespace axes::ecs
