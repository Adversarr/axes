#pragma once
#include <map>
#include <memory>
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
  using ComponentInfoIterator = typename std::vector<ComponentInfo>::iterator;
  /**
   * @brief Query all the entities's ID that obtains component `C`, and insert
   * it into OutputIt
   *
   * @tparam OutputIt output iterator.
   * @param first
   */
  template <typename OutputIt> void QueryAll(OutputIt first) const noexcept;

  void DestroyAll();

  /**
   * @brief Query all the entities that obtains component `C`.
   *
   * @return std::vector<EntityID>
   */
  inline std::vector<EntityID> QueryAll() const noexcept;

  /**
   * @brief Query the component for given entity
   * @param ent
   * @return If the entity exists, returns the pointer to the component.
   *         Otherwise, return NULLPTR.
   */
  Component *Query(EntityID ent) noexcept;

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

  /**
   * @brief Detach the entity's component
   * @note This method will call the default destructor.
   *
   * @param entity
   */
  void DetachComponent(EntityID entity);

  template <typename... Args>
  Component *ReplaceComponent(EntityID ent, Args &&...args);

  template <typename... Args>
  Component *EmplaceComponent(EntityID ent, Args &&...args);

  ComponentManager() noexcept { static PrivateStaticRunner priv_runner; }

  struct _Iterator {
    using iter = std::vector<ComponentInfo>::iterator;

    explicit _Iterator(iter it) : it_(it) {}

    bool operator!=(const _Iterator &rhs) const noexcept { return it_ != rhs.it_; }
    bool operator==(const _Iterator &rhs) const noexcept { return it_ == rhs.it_; }

    _Iterator &operator++() {
      ++it_;
      return *this;
    }

    std::pair<EntityID, Component *> operator*() noexcept {
      return std::make_pair(it_->GetEntityID(),
                            (Component *)it_->GetComponentPtr());
    }

    iter it_;
  };

  // NOLINTBEGIN
  _Iterator begin() { return _Iterator(instances_.begin()); }
  _Iterator end() { return _Iterator(instances_.end()); }
  // NOLINTEND

private:
  uint32_t MakeAvailableChunk();

  template <typename... Args> Component *ConstructAtChunk(Args &&...args);

  ComponentInfoIterator QueryInfo(EntityID ent) noexcept;

  static std::vector<ComponentInfo> instances_;
  static std::vector<uint32_t> non_full_chunk_id_;
  static std::vector<details::Chunk<Component>> chunks_;
  static bool is_sorted_;
};

template <typename Component> bool ComponentManager<Component>::is_sorted_ = false;

template <typename Component>
std::vector<ComponentInfo> ComponentManager<Component>::instances_;

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
  ComponentInfo info(entity, avail_chunk, allocated);
  is_sorted_ &= !(instances_.empty() || instances_.back() < info);
  instances_.push_back(info);
  return allocated;
}

template <typename Component> template <typename... Args>
Component *ComponentManager<Component>::ReplaceComponent(EntityID entity,
                                                         Args &&...args) {
  // Prepare an chunk for storaging, and actually construct the object.
  ComponentInfoIterator comp_info_it = QueryInfo(entity);
  if (comp_info_it == instances_.end()) {
    return nullptr;
  }
  uint32_t chunk_id = comp_info_it->GetChunkID();
  Component *ptr = static_cast<Component *>(comp_info_it->GetComponentPtr());
  ComponentChunk &chunk = chunks_.at(chunk_id);
  ComponentChunk::alloc_trait::destroy(chunk.GetAllocator(), ptr);
  ComponentChunk::alloc_trait::construct(chunk.GetAllocator(), ptr,
                                         std::forward<Args>(args)...);
  return ptr;
}

template <typename Component>
void ComponentManager<Component>::DetachComponent(EntityID entity) {
  ComponentInfoIterator comp_info_it = QueryInfo(entity);
  if (comp_info_it == instances_.end()) {
    return;
  }
  // Locates the chunk, and the internal pointer.
  uint32_t chunk_id = comp_info_it->GetChunkID();
  Component *ptr = static_cast<Component *>(comp_info_it->GetComponentPtr());
  ComponentChunk &chunk = chunks_.at(chunk_id);
  // If the chunk is full before, register it to non-full.
  bool is_full_before_detach = chunk.IsFull();
  chunk.Destroy(ptr);
  if (is_full_before_detach) {
    non_full_chunk_id_.push_back(chunk_id);
  }
  // Remove from info.
  std::swap(*comp_info_it, instances_.back());
  is_sorted_ = false;
  instances_.pop_back();
}

template <typename Component>
typename ComponentManager<Component>::ComponentInfoIterator
ComponentManager<Component>::QueryInfo(EntityID ent) noexcept {
  if (!is_sorted_) {
    // Sort the instances first.
    std::sort(instances_.begin(), instances_.end());
    is_sorted_ = true;
  }
  return std::find(instances_.begin(), instances_.end(), ComponentInfo(ent));
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
  if (it == instances_.end()) {
    return nullptr;
  } else {
    return static_cast<Component *>(it->GetComponentPtr());
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
std::vector<EntityID> ComponentManager<Component>::QueryAll() const noexcept {
  std::vector<EntityID> result;
  result.reserve(instances_.size());
  QueryAll(std::back_inserter(result));
  return result;
}

template <typename Component> template <typename OutputIt>
void ComponentManager<Component>::QueryAll(OutputIt first) const noexcept {
  for (const ComponentInfo &inst : instances_) {
    *first++ = inst.GetEntityID();
  }
}

template <typename Component>
void ComponentManager<Component>::PrivateStaticRunner::Run() const noexcept {
  ComponentManagerInfo info(typeid(Component));
  info.entity_remover_ = [](EntityID ent) -> void {
    ComponentManager<Component> man;
    man.DetachComponent(ent);
  };

  info.querier_ = [](EntityID ent) -> void * {
    return ComponentManager<Component>{}.Query(ent);
  };

  info.query_all_ = []() -> std::vector<EntityID> {
    return ComponentManager<Component>{}.QueryAll();
  };
  info.destroyer_ = []() {
    ComponentManager<Component>{}.DestroyAll();
  };

  World().RegisterComponent<Component>(info);
}

template <typename Component> template <typename... Args>
Component *ComponentManager<Component>::EmplaceComponent(EntityID ent,
                                                         Args &&...args) {
  if (QueryInfo(ent) == instances_.end()) {
    return AttachComponent(ent, std::forward<Args>(args)...);
  } else {
    return ReplaceComponent(ent, std::forward<Args>(args)...);
  }
}

template <typename Component>
void ComponentManager<Component>::DestroyAll() {
  instances_.clear();
  chunks_.clear();
}


}  // namespace axes::ecs
