#include "ax/graph/payload.hpp"
#include "ax/core/entt.hpp"

namespace ax::graph {

namespace details {

struct TypeRegistry {
  absl::flat_hash_map<std::type_index, PayloadDtor> map_;
};

PayloadDtor const& ensure_dtor(std::type_index type, PayloadDtor const &dtor) {
  auto &map = ensure_resource<TypeRegistry>().map_;
  return map.try_emplace(type, dtor).first->second;
}

PayloadDtor const& get_dtor(std::type_index type) {
  auto &map = ensure_resource<TypeRegistry>().map_;
  return map.at(type);
}

}  // namespace details

Payload::~Payload() {
  if (data_ == nullptr) {
    return;
  }
  auto &map = ensure_resource<details::TypeRegistry>().map_;
  if (auto it = map.find(type_); it != map.end()) {
    it->second.dtor_(data_);
  } else {
    AX_CHECK(false) << "Payload type not registered: " << type_.name();
  }
}
Payload Payload::Create(std::type_index t) {
  return Payload(t, details::get_dtor(t).ctor_());
}

}