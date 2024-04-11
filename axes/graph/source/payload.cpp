#include "ax/graph/payload.hpp"
#include "ax/core/entt.hpp"

namespace ax::graph {

namespace details {

void ensure_dtor(std::type_index type, PayloadDtor const &dtor) {
  auto &reg = ensure_resource<TypeRegistry>();
  auto &map = reg.EnsureMeta<PayloadDtor>();
  if (auto it = map.find(type); it == map.end()) {
    map.try_emplace(type, dtor);
  }
}

}  // namespace details

Payload::~Payload() {
  auto &reg = ensure_resource<TypeRegistry>();
  auto const &map = reg.EnsureMeta<PayloadDtor>();
  if (auto it = map.find(type_); it != map.end()) {
    it->second.dtor(data_);
  } else {
    AX_CHECK(false) << "Payload type not registered: " << type_.name();
  }
}

}