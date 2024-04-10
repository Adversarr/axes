#include "ax/graph/node.hpp"

#include <absl/container/flat_hash_map.h>

#include "ax/core/echo.hpp"
#include "ax/core/entt.hpp"
#include "ax/utils/status.hpp"

#if WIN32
#  undef ERROR
#endif

namespace ax::graph {
namespace details {

using constructor_map = absl::flat_hash_map<std::string, NodeConstructor>;
struct wrapper { absl::flat_hash_map<std::string, NodeDescriptor> desc_; };

static inline absl::flat_hash_map<std::string, NodeDescriptor>& ensure_desc() {
  if (auto res = ax::try_get_resource<wrapper>(); res != nullptr) {
    return res->desc_;
  } else {
    return ax::add_resource<wrapper>().desc_;
  }
}
NodeDescriptor const* factory_register(NodeDescriptor desc) {
  auto [it, b] = ensure_desc().try_emplace(desc.name_, desc);
  if (b) {
    AX_DLOG(INFO) << "NodeDescriptor: " << desc.name_ << " registered.";
  }
  return &it->second;
}

}  // namespace details

UPtr<NodeBase> NodeBase::Create(NodeDescriptor const* descript, idx id) {
  auto& cmap = details::ensure_desc();
  auto it = cmap.find(descript->name_);
  if (it != cmap.end()) {
    return (it->second).ctor_(descript, id);
  }
  AX_LOG(ERROR) << "Node " << descript->name_ << " not found.";
  return nullptr;
}

// Some function have default implementation
Status NodeBase::PreApply(idx) { AX_RETURN_OK(); }

Status NodeBase::PostApply(idx) { AX_RETURN_OK(); }

Status NodeBase::OnConstruct() { AX_RETURN_OK(); }

Status NodeBase::OnDestroy() { AX_RETURN_OK(); }

Status NodeBase::CleanUp() { AX_RETURN_OK(); }

NodeBase::NodeBase(NodeDescriptor const* descriptor, idx id) : descriptor_(descriptor), id_(id) {}

}  // namespace ax::graph