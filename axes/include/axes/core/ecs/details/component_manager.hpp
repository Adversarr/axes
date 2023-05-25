#pragma once
#include <absl/container/btree_map.h>

#include <queue>
#include <vector>

#include "axes/core/utils/static_run.hpp"
#include "chunk.hpp"
#include "common.hpp"
#include "world.hpp"

namespace axes::ecs {

template <typename Component> class ComponentManager {
  struct PrivateStaticRunner : public utils::StaticRunner<PrivateStaticRunner> {
    void Run() const noexcept;
  };

public:
  using ComponentChunk = details::Chunk<Component>;
  using ComponentInfoIterator =
      typename absl::btree_map<EntityID, ComponentInfo>::iterator;

  static void DestroyAll();

  /**
   * @brief Query all the entities that obtains component `C`.
   *
   * @return std::vector<EntityID>
   */
  inline static std::vector<EntityID> QueryAll() noexcept;

  /**
   * @brief Query the component for given entity
   * @param ent
   * @return If the entity exists, returns the pointer to the component.
   *         Otherwise, return NULLPTR.
   */
  static Component *Query(EntityID ent) noexcept;

  /**
   * @brief Attach a given component to some entity.
   *
   * @tparam Args
   * @param entity
   * @param args
   * @return
   */
  template <typename... Args>
  static Component *AttachComponent(EntityID entity, Args &&...args);

  /**
   * @brief Detach the entity's component
   * @note This method will call the default destructor.
   *
   * @param entity
   */
  static void DetachComponent(EntityID entity);

  template <typename... Args>
  static Component *ReplaceComponent(EntityID ent, Args &&...args);

  template <typename... Args>
  static Component *AttachOrGet(EntityID ent, Args &&...args);

  ComponentManager() noexcept { static PrivateStaticRunner priv_runner; }

  struct Iterator {
    using iter = absl::btree_map<EntityID, ComponentInfo>::iterator;

    explicit Iterator(iter it) : it_(it) {}

    bool operator!=(const Iterator &rhs) const noexcept { return it_ != rhs.it_; }
    bool operator==(const Iterator &rhs) const noexcept { return it_ == rhs.it_; }

    Iterator &operator++() {
      ++it_;
      return *this;
    }

    std::pair<EntityID, Component *> operator*() noexcept {
      return {it_->first, static_cast<Component *>(it_->second.data_)};
    }

    iter it_;
  };

  // NOLINTBEGIN
  Iterator begin() { return Iterator(entities_.begin()); }
  Iterator end() { return Iterator(entities_.end()); }
  // NOLINTEND

private:
  static uint32_t MakeAvailableChunk();

  template <typename... Args> static Component *ConstructAtChunk(Args &&...args);

  static ComponentInfoIterator QueryInfo(EntityID ent) noexcept;

  static absl::btree_map<EntityID, ComponentInfo> entities_;
  static std::vector<uint32_t> non_full_chunk_id_;
  static std::vector<details::Chunk<Component>> chunks_;
};

template <typename Component>
absl::btree_map<EntityID, ComponentInfo> ComponentManager<Component>::entities_;

template <typename Component>
std::vector<details::Chunk<Component>> ComponentManager<Component>::chunks_;

template <typename Component>
std::vector<uint32_t> ComponentManager<Component>::non_full_chunk_id_;

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
void ComponentManager<Component>::DetachComponent(EntityID entity) {
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
Component *ComponentManager<Component>::Query(EntityID ent) noexcept {
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
std::vector<EntityID> ComponentManager<Component>::QueryAll() noexcept {
  std::vector<EntityID> result;
  result.reserve(entities_.size());
  for (const auto &[k, v] : entities_) {
    result.push_back(k);
  }
  return result;
}

template <typename Component>
void ComponentManager<Component>::PrivateStaticRunner::Run() const noexcept {
  ComponentManagerInfo info(typeid(Component));
  info.detach_ = ComponentManager<Component>::DetachComponent;
  info.query_ = ComponentManager<Component>::Query;
  info.query_all_ = ComponentManager<Component>::QueryAll;
  info.destroy_ = ComponentManager<Component>::DestroyAll;

  World().RegisterComponent(info);
}

template <typename Component> template <typename... Args>
Component *ComponentManager<Component>::AttachOrGet(EntityID ent,
                                                         Args &&...args) {
  if (auto it = QueryInfo(ent); it == entities_.end()) {
    return AttachComponent(ent, std::forward<Args>(args)...);
  } else {
    return static_cast<Component* >(it->second.data_);
  }
}

template <typename Component> void ComponentManager<Component>::DestroyAll() {
  entities_.clear();
  chunks_.clear();
}

}  // namespace axes::ecs
