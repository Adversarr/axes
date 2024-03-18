#pragma once

#include <entt/entity/registry.hpp>
#include <entt/signal/dispatcher.hpp>
#include <entt/signal/sigh.hpp>

#include "common.hpp"  // IWYU pragma: export

namespace ax {

/****************************** Registry ******************************/

/**
 * @brief Get the global registry
 * 
 * @return entt::registry& The global registry
 */
entt::registry& global_registry();

using Entity = entt::entity; ///< Alias for entt::entity

/****************************** Entity ******************************/

/**
 * @brief Create a entity in the global registry
 * 
 * @return Entity The created entity
 */
AX_FORCE_INLINE Entity create_entity() { return global_registry().create(); }


/**
 * @brief Destroy a entity in the global registry
 * 
 * @param entity The entity to destroy
 */
AX_FORCE_INLINE void destroy_entity(Entity entity) { global_registry().destroy(entity); }

/**
 * @brief Check if a entity is alive
 * 
 * @param entity 
 * @return bool 
 */
AX_FORCE_INLINE bool is_alive(Entity entity) { return global_registry().valid(entity); }

using entt::to_integral;
using entt::to_entity;

/****************************** Component ******************************/

/**
 * @brief Create a view of the global registry
 * 
 * @tparam Components The components to view
 * @return auto The view
 */
template <typename... Components> AX_FORCE_INLINE auto view_component() {
  return global_registry().view<Components...>();
}

/**
 * @brief Attach a component to a entity
 * 
 * @tparam Components The component
 * @return auto The view
 */
template <typename T, typename... Args> T& add_component(Entity entity, Args&&... args) {
  return global_registry().emplace<T>(entity, std::forward<Args>(args)...);
}

/**
 * @brief Attach or replace a component to a entity
 * 
 * @tparam T The component
 * @return auto The view
 */
template <typename T, typename ... Args> T& add_or_replace_component(Entity entity, Args&&... args) {
  return global_registry().emplace_or_replace<T>(entity, std::forward<Args>(args)...);
}

/**
 * @brief Get a component from the global registry
 * 
 * @tparam T The component type
 * @param entity The entity
 * @return T& The component
 */
template <typename T> T& get_component(Entity entity) { return global_registry().get<T>(entity); }

/**
 * @brief Get a component from the global registry
 * 
 * @tparam T The component type
 * @param entity The entity
 * @return T* The component
 */
template <typename T> T* try_get_component(Entity entity) {
  return global_registry().try_get<T>(entity);
}

/**
 * @brief Check if a entity has a component
 * 
 * @tparam T The component type
 * @param entity The entity
 * @return true If the entity has the component
 * @return false If the entity does not have the component
 */
template <typename T> bool has_component(Entity entity) {
  return nullptr != try_get_component<T>(entity);
}

/**
 * @brief Remove a component from the global registry
 * 
 * @tparam T The component type
 * @param entity The entity
 * @return true If the component was removed
 * @return false If the component was not removed
 */
template <typename T> bool remove_component(Entity entity) {
  return 0 < global_registry().remove<T>(entity);
}

/****************************** Resource ******************************/

/**
 * @brief Get a resource from the global registry
 * 
 * @tparam T The resource type
 * @return T& The resource
 */
template <typename T> T& get_resource() { return global_registry().ctx().get<T>(); }

/**
 * @brief Get a resource from the global registry
 * 
 * @tparam T The resource type
 * @return T* The resource
 */
template <typename T> T* try_get_resource() { return global_registry().ctx().find<T>(); }

/**
 * @brief Check if a resource is in the global registry
 * 
 * @tparam T The resource type
 * @return true If the resource is in the global registry
 * @return false If the resource is not in the global registry
 */
template <typename T, typename... Args> T& add_resource(Args&&... args) {
  return global_registry().ctx().emplace<T>(std::forward<Args>(args)...);
}

/**
 * @brief Remove a resource from the global registry
 * 
 * @tparam T The resource type
 */
template <typename T> void erase_resource() { global_registry().ctx().erase<T>(); }

/****************************** Signal Handlers ******************************/

/**
 * @brief Get the global dispatcher
 * 
 * @return entt::dispatcher& The global dispatcher
 */
entt::dispatcher& global_dispatcher();

/**
 * @brief Emit a event to the global dispatcher, Run immediately
 * 
 * @tparam Event The event type
 * @param event The event
 */
template <typename Event> void emit(Event&& event) {
  global_dispatcher().trigger(std::forward<Event>(event));
}

/**
 * @brief Emit a event to the global dispatcher, Run Differed
 * 
 * @tparam Event The event type
 * @param event The event
 */
template <typename Event> void emit_enqueue(Event&& event) {
  global_dispatcher().enqueue(std::forward<Event>(event));
}

/**
 * @brief Trigger the global dispatcher, Run immediately
 * 
 * @tparam Event The event type
 * @param event The event
 */
template <typename Event> void trigger_queue() { global_dispatcher().update<Event>(); }

/**
 * @brief Trigger all the events in the global dispatcher, Run immediately
 */
AX_FORCE_INLINE void trigger_queue() { global_dispatcher().update(); }

/**
 * @brief Connect a event to the global dispatcher
 * 
 * @tparam Event The event type
 * @tparam Candidate The function to connect
 * @tparam Args The arguments
 * @return entt::connection The connection
 */
template <typename Event, auto Candidate, typename... Args>
entt::connection connect(Args&&... args) {
  return global_dispatcher().sink<Event>().template connect<Candidate>(std::forward<Args>(args)...);
}

/**
 * @brief Disconnect a event from the global dispatcher
 * 
 * @tparam Event The event type
 * @tparam Candidate The function to disconnect
 * @tparam Args The arguments
 */
template <typename Event, auto Candidate, typename... Args> void disconnect(Args&&... args) {
  global_dispatcher().sink<Event>().template disconnect<Candidate>(std::forward<Args>(args)...);
}

}  // namespace ax
