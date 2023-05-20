#include "acore/ecs/details/resourcemanager.hpp"

namespace axes::ecs {

std::unordered_map<std::type_index, ResourceManager::ResourceMeta>
    ResourceManager::resources_;

void ResourceManager::DestroyGlobal() {
  for (auto [k, v]: resources_) {
    v.deleter_(v.data_);
    v.data_ = nullptr;
  }
}

}