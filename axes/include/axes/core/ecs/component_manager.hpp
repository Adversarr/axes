#pragma once
#include <absl/container/btree_map.h>

#include <mutex>
#include <queue>
#include <vector>

#include "chunk.hpp"
#include "common.hpp"
#include "world.hpp"

namespace axes::ecs {

template <typename Component> struct ComponentIterator {
  using iter = absl::btree_map<EntityID, ComponentInfo>::iterator;

  explicit ComponentIterator(iter it) : it_(it) {}

  bool operator!=(const ComponentIterator &rhs) const noexcept {
    return it_ != rhs.it_;
  }
  bool operator==(const ComponentIterator &rhs) const noexcept {
    return it_ == rhs.it_;
  }

  ComponentIterator &operator++() {
    ++it_;
    return *this;
  }

  std::pair<EntityID, Component *> operator*() noexcept {
    return {it_->first, static_cast<Component *>(it_->second.data_)};
  }

  iter it_;
};

template <typename Component> class ComponentManager {
public:
  using ComponentChunk = details::Chunk<Component>;
  using ComponentInfoIterator =
      typename absl::btree_map<EntityID, ComponentInfo>::iterator;

  void DestroyAll() { DestroyAllInternal(); }

  void DetachComponent(EntityID ent) { DetachComponentInternal(ent); }

  std::vector<EntityID> QueryAll() { return QueryAllInternal(); }

  Component *Query(EntityID ent) { return QueryInternal(ent); }

  /**
   * @brief Attach a given component to some entity.
   *
   * @tparam Args
   * @param entity
   * @param args
   * @return
   */
  template <typename... Args>
  Component *AttachComponent(EntityID entity, Args &&...args);

  template <typename... Args>
  Component *ReplaceComponent(EntityID ent, Args &&...args);

  template <typename... Args> Component *AttachOrGet(EntityID ent, Args &&...args);

  ComponentManager() noexcept;

  // NOLINTBEGIN
  ComponentIterator<Component> begin() {
    return ComponentIterator<Component>(entities_.begin());
  }

  ComponentIterator<Component> end() {
    return ComponentIterator<Component>(entities_.end());
  }
  // NOLINTEND

private:
  static void DestroyAllInternal();

  /**
   * @brief Query all the entities that obtains component `C`.
   *
   * @return std::vector<EntityID>
   */
  static inline std::vector<EntityID> QueryAllInternal() noexcept;

  /**
   * @brief Query the component for given entity
   * @param ent
   * @return If the entity exists, returns the pointer to the component.
   *         Otherwise, return NULLPTR.
   */
  static Component *QueryInternal(EntityID ent) noexcept;

  /**
   * @brief Detach the entity's component
   * @note This method will call the default destructor.
   *
   * @param entity
   */
  static void DetachComponentInternal(EntityID entity);

  static uint32_t MakeAvailableChunk();

  template <typename... Args> static Component *ConstructAtChunk(Args &&...args);

  static ComponentInfoIterator QueryInfo(EntityID ent) noexcept;
  static absl::btree_map<EntityID, ComponentInfo> entities_;
  static std::vector<uint32_t> non_full_chunk_id_;
  static std::vector<details::Chunk<Component>> chunks_;
  static std::once_flag oflag_;
};

template <typename Component>
ComponentManager<Component>::ComponentManager() noexcept {
  std::call_once(oflag_, []() {
    ComponentManagerInfo info(typeid(Component));
    info.detach_ = ComponentManager<Component>::DetachComponentInternal;
    info.query_ = ComponentManager<Component>::QueryInternal;
    info.query_all_ = ComponentManager<Component>::QueryAllInternal;
    info.destroy_ = ComponentManager<Component>::DestroyAllInternal;
    World{}.RegisterComponent(info);
  });
}

template <typename Component>
absl::btree_map<EntityID, ComponentInfo> ComponentManager<Component>::entities_;

template <typename Component>
std::vector<details::Chunk<Component>> ComponentManager<Component>::chunks_;

template <typename Component>
std::vector<uint32_t> ComponentManager<Component>::non_full_chunk_id_;

template <typename Component> std::once_flag ComponentManager<Component>::oflag_;

template <typename Component> template <typename... Args>
Component *ComponentManager<Component>::AttachComponent(EntityID entity,
                                                        Args &&...args) {
  // Prepare an chunk for storaging, and actually construct the object.
  auto avail_chunk = MakeAvailableChunk();
  Component *allocated = ConstructAtChunk(std::forward<Args>(args)...);
  ComponentInfo info(avail_chunk, allocated);
  entities_.insert({entity, info});
  return allocated;
}

template <typename Component> template <typename... Args>
Component *ComponentManager<Component>::ReplaceComponent(EntityID entity,
                                                         Args &&...args) {
  // Prepare an chunk for storaging, and actually construct the object.
  ComponentInfoIterator comp_info_it = QueryInfo(entity);
  if (comp_info_it == entities_.end()) {
    return nullptr;
  }
  Component *ptr = static_cast<Component *>(comp_info_it->second.data_);
  std::destroy_at(ptr);
  std::construct_at(ptr, std::forward<Args>(args)...);
  return ptr;
}

template <typename Component>
void ComponentManager<Component>::DetachComponentInternal(EntityID entity) {
  ComponentInfoIterator comp_info_it = QueryInfo(entity);
  if (comp_info_it == entities_.end()) {
    return;
  }
  // Locates the chunk, and the internal pointer.
  uint32_t chunk_id = comp_info_it->second.chunk_id_;
  Component *ptr = static_cast<Component *>(comp_info_it->second.data_);
  ComponentChunk &chunk = chunks_.at(chunk_id);
  // If the chunk is full before, register it to non-full.
  bool is_full_before_detach = chunk.IsFull();
  chunk.Destroy(ptr);
  if (is_full_before_detach) {
    non_full_chunk_id_.push_back(chunk_id);
  }
  // Remove from info.
  entities_.erase(comp_info_it);
}

template <typename Component>
typename ComponentManager<Component>::ComponentInfoIterator
ComponentManager<Component>::QueryInfo(EntityID ent) noexcept {
  return entities_.find(ent);
}

template <typename Component>
uint32_t ComponentManager<Component>::MakeAvailableChunk() {
  if (non_full_chunk_id_.empty()) {
    non_full_chunk_id_.push_back(static_cast<uint32_t>(chunks_.size()));
    chunks_.emplace_back();
  }
  return non_full_chunk_id_.front();
}

template <typename Component>
Component *ComponentManager<Component>::QueryInternal(EntityID ent) noexcept {
  auto it = QueryInfo(ent);
  if (it == entities_.end()) {
    return nullptr;
  } else {
    return static_cast<Component *>(it->second.data_);
  }
}

template <typename Component> template <typename... Args>
Component *ComponentManager<Component>::ConstructAtChunk(Args &&...args) {
  ComponentChunk &c = chunks_[non_full_chunk_id_.front()];
  Component *p = c.Create(std::forward<Args>(args)...);
  if (c.IsFull()) {
    std::swap(non_full_chunk_id_.front(), non_full_chunk_id_.back());
    non_full_chunk_id_.pop_back();
  }
  return p;
}

template <typename Component>
std::vector<EntityID> ComponentManager<Component>::QueryAllInternal() noexcept {
  std::vector<EntityID> result;
  result.reserve(entities_.size());
  for (const auto &[k, v] : entities_) {
    result.push_back(k);
  }
  return result;
}

template <typename Component> template <typename... Args>
Component *ComponentManager<Component>::AttachOrGet(EntityID ent, Args &&...args) {
  if (auto it = QueryInfo(ent); it == entities_.end()) {
    return AttachComponent(ent, std::forward<Args>(args)...);
  } else {
    return static_cast<Component *>(it->second.data_);
  }
}

template <typename Component>
void ComponentManager<Component>::DestroyAllInternal() {
  entities_.clear();
  chunks_.clear();
}

}  // namespace axes::ecs
