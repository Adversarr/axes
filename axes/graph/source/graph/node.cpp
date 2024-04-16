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

struct wrapper {
  absl::flat_hash_map<std::string, NodeDescriptor> desc_;
  std::vector<std::string> node_names_;
  bool is_sorted = true;
};

static inline wrapper& ensure_wrapper() {
  return ensure_resource<wrapper>();
}
NodeDescriptor const* factory_register(NodeDescriptor desc) {
  auto &w = ensure_wrapper();
  auto [it, b] = w.desc_.try_emplace(desc.name_, desc);
  if (b) {
    AX_DLOG(INFO) << "NodeDescriptor: " << desc.name_ << " registered.";
    ensure_resource<wrapper>().node_names_.emplace_back(desc.name_);
    w.is_sorted = false;
  }
  return &it->second;
}

std::vector<std::string> const& get_node_names() {
  auto &w = ensure_wrapper();
  if (!w.is_sorted) {
    std::sort(w.node_names_.begin(), w.node_names_.end());
    w.is_sorted = true;
  }
  return w.node_names_;
}

NodeDescriptor const* get_node_descriptor(const char* name) {
  auto& cmap = ensure_wrapper().desc_;
  auto it = cmap.find(name);
  if (it != cmap.end()) {
    return &it->second;
  }
  return nullptr;
}

}  // namespace details

UPtr<NodeBase> NodeBase::Create(NodeDescriptor const* descript, idx id) {
  auto& cmap = details::ensure_wrapper().desc_;
  auto it = cmap.find(descript->name_);
  if (it != cmap.end()) {
    return (it->second).ctor_(descript, id);
  }
  AX_LOG(ERROR) << "Node " << descript->name_ << " not found.";
  return nullptr;
}

// Some function have default implementation
Status NodeBase::Apply(ax::idx) { AX_RETURN_OK(); }

Status NodeBase::PreApply() { AX_RETURN_OK(); }

Status NodeBase::PostApply() { AX_RETURN_OK(); }

Status NodeBase::OnConnect(idx) { AX_RETURN_OK(); }

Status NodeBase::OnConstruct() { AX_RETURN_OK(); }

void NodeBase::OnDestroy() {}

void NodeBase::CleanUp() {}

boost::json::object NodeBase::Serialize() const {
  return {};
}

void NodeBase::Deserialize(boost::json::object const&) {
  // nothing to do.
}


NodeBase::NodeBase(NodeDescriptor const* descriptor, idx id) : descriptor_(descriptor), id_(id) {}

}  // namespace ax::graph
