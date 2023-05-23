#pragma once
#include <functional>

#include "axes/core/common.hpp"
#include <typeindex>
namespace axes::ecs {

using EntityID = UInt32;
using ComponentID = UInt32;

class World;
class EntityManager;
template <typename Type> class Resource;
template <typename Component> class ComponentManager;

/**
 * @brief Single Component with Entity
 */
class ComponentInfo {
public:
  inline bool operator<(const ComponentInfo &rhs) const noexcept {
    return belonging_ < rhs.belonging_;
  }

  inline bool operator==(const ComponentInfo &rhs) const noexcept {
    return belonging_ == rhs.belonging_;
  }

  /**
   * @brief Returns the entity id.
   *
   * @return
   */
  inline EntityID GetEntityID() const noexcept { return belonging_; }

  /**
   * @brief Returns the component's pointer.
   *
   * @return
   */
  inline void *GetComponentPtr() const noexcept { return data_; }

  inline uint32_t GetChunkID() const noexcept { return chunk_id_; }

  /**
   * @brief Create a new component with given component ptr.
   * @param ent
   * @param ptr
   */
  explicit ComponentInfo(EntityID ent, uint32_t chunk_id = 0,
                         void *component_ptr = nullptr)
      : belonging_{ent}, chunk_id_{chunk_id}, data_{component_ptr} {}

  // Allow copy.
  ComponentInfo(const ComponentInfo &) = default;

  ComponentInfo& operator=(const ComponentInfo& ) = default;

private:
  // 16 Byte storage required.
  EntityID belonging_{0};
  uint32_t chunk_id_{0};
  void *data_{nullptr};
};

struct ComponentManagerInfo {
  std::type_index ti_;
  /**
   * @brief EntityID -> Component Pointer.
   */
  std::function<void *(EntityID)> querier_;

  /**
   * @brief () -> [EntityID].
   */
  std::function<std::vector<EntityID>()> query_all_;

  /**
   * @brief Removes a given entity from component manager.
   */
  std::function<void(EntityID)> entity_remover_;

  std::function<void()> destroyer_;
  explicit ComponentManagerInfo(std::type_index ti): ti_(ti) {}
};
}  // namespace axes
