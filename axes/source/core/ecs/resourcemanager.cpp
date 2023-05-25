#include "axes/core/ecs/details/resource_manager.hpp"

namespace axes::ecs {

absl::flat_hash_map<std::type_index, ResourceManager::ResourceMeta>
    ResourceManager::resources_;

void ResourceManager::WorldDestroy() {
  for (auto [k, v] : resources_) {
    v.destroyer_(v.data_);
    std::free(v.data_);
    v.data_ = nullptr;
    std::cout << "Destroy Resource: " << k.name() << std::endl;
  }
  resources_.clear();
}

}  // namespace axes::ecs
