#include "ax/graph/node.hpp"

#include "ax/core/entt.hpp"
#include "ax/core/logging.hpp"

namespace ax::graph {
namespace details {

using constructor_map = std::map<std::string, NodeConstructor>;

struct wrapper {
  std::map<std::string, NodeDescriptor> desc_;
  std::vector<std::string> node_names_;
  bool is_sorted = true;
};

static inline wrapper& ensure_wrapper() { return ensure_resource<wrapper>(); }
NodeDescriptor const* factory_register(NodeDescriptor desc) {
  auto& w = ensure_wrapper();
  auto [it, b] = w.desc_.try_emplace(desc.name_, desc);
  if (b) {
    AX_TRACE("NodeDescriptor: {} registered.", desc.name_);
    ensure_resource<wrapper>().node_names_.emplace_back(desc.name_);
    w.is_sorted = false;
  }
  return &it->second;
}

std::vector<std::string> const& get_node_names() {
  auto& w = ensure_wrapper();
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

std::unique_ptr<NodeBase> NodeBase::Create(NodeDescriptor const* descript, size_t id) {
  auto& cmap = details::ensure_wrapper().desc_;
  auto it = cmap.find(descript->name_);
  if (it != cmap.end()) {
    return (it->second).ctor_(descript, id);
  }
  AX_ERROR("Node {} not found.", descript->name_);
  return nullptr;
}

// Some function have default implementation
void NodeBase::Apply(size_t) {}

void NodeBase::PreApply() {}

void NodeBase::PostApply() {}

void NodeBase::OnConnect(size_t) {}
void NodeBase::CleanUp() noexcept {}

boost::json::object NodeBase::Serialize() const { return {}; }

void NodeBase::Deserialize(boost::json::object const&) {
  // nothing to do.
}

NodeBase::NodeBase(NodeDescriptor const* descriptor, size_t id)
    : descriptor_(descriptor), id_(id) {}

}  // namespace ax::graph
