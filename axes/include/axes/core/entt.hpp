#pragma once

#include <entt/entity/registry.hpp>
#include <entt/signal/dispatcher.hpp>
#include <entt/signal/sigh.hpp>

#include "common.hpp"  // IWYU pragma: export

namespace ax {

entt::registry& get_registry();

using Entity = entt::entity;

/****************************** Entity ******************************/

AX_FORCE_INLINE entt::entity create_entity() { return get_registry().create(); }

/****************************** Component ******************************/

template <typename... Components> AX_FORCE_INLINE auto view_component() {
  return get_registry().view<Components...>();
}

template <typename T, typename... Args> T& add_component(Entity entity, Args&&... args) {
  return get_registry().emplace<T>(entity, std::forward<Args>(args)...);
}

template <typename T> T& get_component(Entity entity) { return get_registry().get<T>(entity); }

template <typename T> T* try_get_component(Entity entity) {
  return get_registry().try_get<T>(entity);
}

template <typename T> bool has_component(Entity entity) {
  return nullptr != try_get_component<T>(entity);
}

template <typename T> bool remove_component(Entity entity) {
  return 0 < get_registry().remove<T>(entity);
}

/****************************** Resource ******************************/

template <typename T> T& get_resource() { return get_registry().ctx().get<T>(); }

template <typename T> T* try_get_resource() { return get_registry().ctx().find<T>(); }

template <typename T, typename... Args> T& add_resource(Args&&... args) {
  return get_registry().ctx().emplace<T>(std::forward<Args>(args)...);
}

template <typename T> void erase_resource() { get_registry().ctx().erase<T>(); }

/****************************** Signal Handlers ******************************/

entt::dispatcher& get_dispatcher();

template <typename Event> void emit(Event&& event) {
  get_dispatcher().trigger(std::forward<Event>(event));
}

template <typename Event> void emit_enqueue(Event&& event) {
  get_dispatcher().enqueue(std::forward<Event>(event));
}

template <typename Event> void trigger_queue() { get_dispatcher().update<Event>(); }

inline void trigger_queue() { get_dispatcher().update(); }

template <typename Event, auto Candidate, typename... Args>
entt::connection connect(Args&&... args) {
  return get_dispatcher().sink<Event>().template connect<Candidate>(std::forward<Args>(args)...);
}

template <typename Event, auto Candidate, typename... Args> void disconnect(Args&&... args) {
  get_dispatcher().sink<Event>().template disconnect<Candidate>(std::forward<Args>(args)...);
}

}  // namespace ax
