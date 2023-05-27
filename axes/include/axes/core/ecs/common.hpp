#pragma once
#include <functional>
#include <typeindex>

#include "axes/core/common.hpp"
namespace axes::ecs {

using EntityID = UInt32;

class World;
class EntityManager;
template <typename Type> class Rc;
template <typename Component> class ComponentManager;

/**
 * @brief Single Component with Entity
 */
struct ComponentInfo {
  /**
   * @brief Create a new component with given component ptr.
   * @param ent
   * @param ptr
   */
  ComponentInfo(uint32_t chunk_id, void *component_ptr)
      : chunk_id_{chunk_id}, data_{component_ptr} {}
  // 16 Byte storage required.
  uint32_t chunk_id_;
  void *data_;
};

struct ComponentManagerInfo {
  std::type_index ti_;
  /**
   * @brief EntityID -> Component Pointer.
   */
  std::function<void *(EntityID)> query_;

  /**
   * @brief () -> [EntityID].
   */
  std::function<std::vector<EntityID>()> query_all_;

  /**
   * @brief Removes a given entity from component manager.
   */
  std::function<void(EntityID)> detach_;

  std::function<void()> destroy_;

  explicit ComponentManagerInfo(std::type_index ti) : ti_(ti) {}
};

}  // namespace axes::ecs
