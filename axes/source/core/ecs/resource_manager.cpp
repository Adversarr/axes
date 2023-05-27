#include "axes/core/ecs/resource_manager.hpp"

namespace axes::ecs {

absl::flat_hash_map<std::type_index, RMan::RcStorage> RMan::resources_;

void RMan::DestroyAll() {
  for (auto [k, v] : resources_) {
    v.destroyer_(v.data_);
    std::free(v.data_);
    v.data_ = nullptr;
    std::cout << "Destroy Resource: " << k.name() << std::endl;
  }
  resources_.clear();
}

void RMan::Destroy(std::type_index ti) {
  auto it = resources_.find(ti);
  if (it == resources_.end()) {
    return;
  }

  it->second.destroyer_(it->second.data_);
  if (it->second.data_) {
    free(it->second.data_);
    it->second.data_ = nullptr;
  }
  // XXX: Whether to destroy the whole ResourceStorage object?
  resources_.erase(it);
}

void RMan::Publish(std::type_index ti) {
  auto it = resources_.find(ti);
  if (it == resources_.end()) {
    return;
  }

  for (auto& f : it->second.on_upd_) {
    f();
  }
}

void RMan::Subscribe(std::type_index ti, std::function<void(void)> upd) {
  auto it = resources_.find(ti);
  if (it == resources_.end()) {
    return;
  }
  it->second.on_upd_.push_back(std::move(upd));
}
void RMan::SetDestroyer(std::type_index ti, std::function<void(void*)> d) {
  auto it = resources_.find(ti);
  if (it == resources_.end()) {
    return;
  }
  it->second.destroyer_ = d;
}

}  // namespace axes::ecs
