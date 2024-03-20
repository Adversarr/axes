#include "axes/utils/meta.hpp"

#include <absl/container/flat_hash_map.h>
#include "axes/core/entt.hpp"
#include "axes/core/echo.hpp"

namespace ax::utils {

using MetaRegistry = absl::flat_hash_map<std::type_index, TypeMeta>;
namespace details {
void initialize_type_meta_registry() {
  if (auto ptr = try_get_resource<MetaRegistry>(); ptr != nullptr) {
    AX_LOG(WARNING) << "TypeMeta registry already initialized";
    return;
  }
  add_resource<MetaRegistry>(std::make_shared<MetaRegistry>());
}

}

  TypeMeta const* get_type_meta(std::type_index const& type_index) {
    auto const& registry = get_resource<MetaRegistry>();
    auto const it = registry.find(type_index);
    if (it != registry.end()) {
      return &it->second;
    }
    return nullptr;
  }

  TypeMeta const* register_type_meta(TypeMeta const& meta) {
    auto& registry = get_resource<MetaRegistry>();
    auto const it = registry.find(meta.index_);
    if (it != registry.end()) {
      return &it->second;
    }
    auto && result = registry.emplace(meta.index_, meta);
    return &(result.first->second);
  }

}  // namespace ax::utils