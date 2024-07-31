#include "ax/graph/graph.hpp"

#include <boost/describe/enum.hpp>
#include <boost/scope_exit.hpp>
#include <map>
#include <range/v3/view.hpp>
#include <set>

#include "ax/core/excepts.hpp"

#define SPDLOG_ACTIVE_LEVEL SPDLOG_LEVEL_TRACE
#include "ax/core/logging.hpp"
#include "ax/graph/node.hpp"
#include "ax/utils/enum_refl.hpp"

namespace ax::graph {

BOOST_DEFINE_ENUM(UuidInferType, kNode, kPin, kSocket, kInvalid);

struct UuidUnderlyingInfo {
  UuidInferType const type_;
  size_t real_id_;
};

struct Graph::Impl {
  // Uuid.
  std::vector<size_t> uuid_reuse_node_, uuid_reuse_pin_, uuid_reuse_socket_;
  size_t uuid_next_ = 0;
  std::vector<UuidUnderlyingInfo> uuid_info_;
  std::map<id_t, id_t> pin_to_node_;
  std::map<id_t, std::set<id_t>> node_to_sockets_;

  // Container for graph elements.
  std::vector<std::unique_ptr<NodeBase>> nodes_;
  std::vector<Socket> sockets_;
  std::vector<Payload> payloads_;

  Impl() {
    uuid_info_.emplace_back(UuidUnderlyingInfo{kInvalid, INVALID_ID});
    uuid_next_ = 1;
  }

  size_t UuidStepWithReuse(UuidInferType underlying, std::vector<size_t>& reuse) {
    size_t id = 0;
    if (reuse.empty()) {
      id = uuid_next_++;
    } else {
      id = reuse.back();
      reuse.pop_back();
    }

    if (id >= uuid_info_.size()) {
      uuid_info_.emplace_back(UuidUnderlyingInfo{underlying, INVALID_ID});
    }
    return id;
  }

  size_t UuidStep(UuidInferType underlying) {
    size_t id = INVALID_ID;
    switch (underlying) {
      case kNode:
        id = UuidStepWithReuse(underlying, uuid_reuse_node_);
        break;
      case kPin:
        id = UuidStepWithReuse(underlying, uuid_reuse_pin_);
        break;
      case kSocket:
        id = UuidStepWithReuse(underlying, uuid_reuse_socket_);
        break;
      default:
        AX_CHECK(underlying != underlying, "Internal Logic Error!");
    }
    return id;
  }

  void EmplaceNode(std::unique_ptr<NodeBase> n) {
    AX_CHECK(n->id_ < uuid_info_.size(), "Internal Logic Error!");
    auto& uinfo = uuid_info_[n->id_];
    node_to_sockets_.insert({n->id_, std::set<id_t>{}});
    if (uinfo.real_id_ == INVALID_ID) {
      uinfo.real_id_ = nodes_.size();
      nodes_.push_back(std::move(n));
    } else {
      AX_CHECK(nodes_[uinfo.real_id_].get() == nullptr,
               "You are trying to overwrite an existing node!");
      nodes_[uinfo.real_id_] = std::move(n);
    }

    for (auto& pin : nodes_[uinfo.real_id_]->inputs_) {
      pin_to_node_[pin.id_] = nodes_[uinfo.real_id_]->id_;
    }
    for (auto& pin : nodes_[uinfo.real_id_]->outputs_) {
      pin_to_node_[pin.id_] = nodes_[uinfo.real_id_]->id_;
    }
  }

  void RemoveNode(id_t i) {
    auto& n = nodes_[i];
    auto uid = n->id_;
    n.reset(nullptr);
    UuidRecycle(uid);
  }

  void EmplaceSocket(Socket s) {
    AX_CHECK(s.id_ <= uuid_info_.size(), "Internal Logic Error!");
    auto& uinfo = uuid_info_[s.id_];
    if (uinfo.real_id_ == INVALID_ID) {
      uinfo.real_id_ = sockets_.size();
      sockets_.push_back(s);
    } else {
      AX_CHECK(sockets_[uinfo.real_id_].id_ == INVALID_ID,
               "You are trying to overwrite an existing socket!");
      sockets_[uinfo.real_id_] = s;
    }
    node_to_sockets_[s.input_->node_id_].insert(s.id_);
    node_to_sockets_[s.output_->node_id_].insert(s.id_);
  }

  void RemoveSocket(id_t i) {
    auto& s = sockets_[i];
    auto uid = s.id_;
    node_to_sockets_[s.input_->node_id_].erase(uid);
    node_to_sockets_[s.output_->node_id_].erase(uid);
    s.id_ = INVALID_ID;
    AX_TRACE("Socket <uid={}> removed.", i);
    UuidRecycle(uid);
  }

  void UuidRecycle(size_t id) {
    auto& info = uuid_info_[id];
    AX_TRACE("Reusing {} type: {}", id, utils::reflect_name(info.type_).value_or("Unknown"));
    switch (info.type_) {
      case kNode:
        uuid_reuse_node_.push_back(id);
        break;
      case kPin:
        uuid_reuse_pin_.push_back(id);
        break;
      case kSocket:
        uuid_reuse_socket_.push_back(id);
        break;
      default:
        AX_DCHECK(false, != info.type_, "Internal Logic Error!");
    }
  }
};

Graph::Graph() { impl_ = std::make_unique<Impl>(); }
Graph::~Graph() { Clear(); }

NodeBase* Graph::AddNode(NodeDescriptor const* descriptor) {
  size_t id = impl_->UuidStep(kNode);
  auto n = NodeBase::Create(descriptor, id);
  std::vector<size_t> ids_used;
  bool build_failed = false;

  BOOST_SCOPE_EXIT(build_failed, ids_used, &impl_) {
    if_unlikely(build_failed) {
      for (auto id : ids_used) {
        impl_->UuidRecycle(id);
      }
    }
  }
  BOOST_SCOPE_EXIT_END;

  // Not found.
  if_unlikely(!n) {
    build_failed = true;
    return nullptr;
  }

  // Construct the node Input and Output.
  n->inputs_.reserve(descriptor->inputs_.size());
  n->outputs_.reserve(descriptor->outputs_.size());
  n->output_payloads_.reserve(descriptor->outputs_.size());

  for (auto const& [count, in_desc] : ranges::views::enumerate(descriptor->inputs_)) {
    size_t uid = impl_->UuidStep(kPin);
    ids_used.push_back(uid);
    n->inputs_.emplace_back(Pin{n->id_, count, true, &in_desc, uid, nullptr});
    AX_TRACE("Input Pin {} added. <uid={}>", in_desc.name_, uid);
  }
  for (auto const& [count, out_desc] : ranges::views::enumerate(descriptor->outputs_)) {
    size_t uid = impl_->UuidStep(kPin);
    ids_used.push_back(uid);
    n->output_payloads_.emplace_back(Payload::Create(out_desc.type_));
    n->outputs_.emplace_back(
        Pin{n->id_, count, false, &out_desc, uid, &n->output_payloads_.back()});
    AX_TRACE("Output Pin {} added. <uid={}>", out_desc.name_, uid);
  }

  // Everything is ok.
  NodeBase* node = n.get();
  impl_->EmplaceNode(std::move(n));
  AX_TRACE("Node {} added. <uid={}>", descriptor->name_, node->id_);
  return node;
}

NodeBase* Graph::GetNode(size_t id) {
  auto const& uinfo = impl_->uuid_info_;
  if (id >= uinfo.size()) {
    throw make_out_of_range("Node not found!");
  }

  const auto& info = uinfo[id];
  if (info.type_ != kNode) {
    return nullptr;
  }
  AX_DCHECK(info.real_id_ < impl_->nodes_.size(), "Internal Logic Error!");
  return impl_->nodes_[info.real_id_].get();
}

NodeBase const* Graph::GetNode(size_t id) const { return const_cast<Graph*>(this)->GetNode(id); }

bool Graph::RemoveNode(NodeBase* n) { return RemoveNode(n->id_); }

bool Graph::RemoveNode(size_t id) {
  auto const& uinfo = impl_->uuid_info_;
  if (id >= uinfo.size()) {
    throw make_out_of_range("Node not found!");
  }

  const auto& info = uinfo[id];
  if (info.type_ != kNode) {
    throw make_invalid_argument("Invalid node id, not a node! {}",
                                utils::reflect_name(info.type_).value_or("???"));
  }

  auto* node = impl_->nodes_[info.real_id_].get();

  for (auto& pin : node->inputs_) {
    impl_->UuidRecycle(pin.id_);
  }
  for (auto& pin : node->outputs_) {
    impl_->UuidRecycle(pin.id_);
  }
  node->CleanUp();

  if (auto it = impl_->node_to_sockets_.find(id); it != impl_->node_to_sockets_.end()) {
    auto copy = it->second;
    for (id_t sock_id : copy) {
      RemoveSocket(sock_id);
    }
    impl_->node_to_sockets_.erase(id);
  }
  impl_->RemoveNode(info.real_id_);
  AX_TRACE("Node <uid={}>, <real_id={}> removed.", id, info.real_id_);
  return true;
}

Socket* Graph::AddSocket(Pin* input, Pin* output) {
  if (!input || !output) {
    return nullptr;
  }
  if (!CanConnectSocket(input, output)) {
    return nullptr;
  }
  if (Socket* existing_socket = GetSocket(output)) {
    AX_TRACE("Socket on {} already exists! Remove before add a new one", output->node_id_);
    RemoveSocket(existing_socket);
  }

  id_t sock_id = impl_->UuidStep(kSocket);
  Socket socket{sock_id, input, output};

  NodeBase* in_node = GetNode(input->node_id_);
  NodeBase* out_node = GetNode(output->node_id_);

  socket.output_->payload_ = &in_node->output_payloads_[input->node_io_index_];

  // Set the input pin to the socket.
  impl_->EmplaceSocket(socket);
  output->socket_in_id_ = sock_id;
  AX_TRACE("Socket <uid={}> added. <{}--{}>", sock_id, input->id_, output->id_);
  out_node->OnConnect(socket.output_->node_io_index_);
  return &(impl_->sockets_[impl_->uuid_info_[sock_id].real_id_]);
}

Socket* Graph::AddSocket(id_t input_pin, id_t output_pin) {
  Pin *input = GetPin(input_pin), *output = GetPin(output_pin);
  if (!input || !output) {
    return nullptr;
  }
  return AddSocket(input, output);
}

Socket* Graph::AddSocket(id_t left, id_t input_pin, id_t right, id_t output_pin) {
  NodeBase* data_in_node = GetNode(left);
  NodeBase* data_out_node = GetNode(right);
  // The node does not exist
  if (!data_in_node || !data_out_node) {
    AX_ERROR("Node not found!");
    return nullptr;
  }

  if (input_pin >= data_in_node->outputs_.size() || output_pin >= data_out_node->inputs_.size()) {
    AX_ERROR("Pin index out of range!");
    return nullptr;
  }

  return AddSocket(&(data_in_node->outputs_[input_pin]), &(data_out_node->inputs_[output_pin]));
}

bool Graph::CanConnectSocket(Pin const* input, Pin const* output) const {
  if (!input || !output) {
    throw make_invalid_argument("Invalid input or output pin!");
    return false;
  }
  if (input->is_input_ || !output->is_input_) {
    AX_WARN("Link is only available when O->I");
    return false;
  }
  if (input->descriptor_->type_ != output->descriptor_->type_) {
    AX_WARN("Link is only available when the type is the same: {} != {}",
            input->descriptor_->type_.name(), output->descriptor_->type_.name());
    return false;
  }
  return true;
}

bool Graph::CanConnectSocket(id_t input_pin, id_t output_pin) const {
  Pin const *input = GetPin(input_pin), *output = GetPin(output_pin);
  return CanConnectSocket(input, output);
}

bool Graph::CanConnectSocket(id_t input_node, id_t input_pin, id_t output_node,
                             id_t output_pin) const {
  NodeBase const* data_in_node = GetNode(input_node);
  NodeBase const* data_out_node = GetNode(output_node);
  if (data_in_node == nullptr || data_out_node == nullptr) {
    throw make_invalid_argument("Node not found!");
  }

  if (input_pin >= data_in_node->outputs_.size()) {
    throw make_out_of_range("Pin index out of range!");
  }
  if (output_pin >= data_out_node->inputs_.size()) {
    throw make_out_of_range("Pin index out of range!");
  }

  return CanConnectSocket(&(data_in_node->outputs_[input_pin]),
                          &(data_out_node->inputs_[output_pin]));
}

Socket* Graph::GetSocket(id_t id) {
  auto const& uinfo = impl_->uuid_info_;
  if (id >= uinfo.size() || id < 0) {
    throw make_out_of_range("Socket not found!");
  }

  const auto& info = uinfo[id];
  if (info.type_ != kSocket) {
    throw make_invalid_argument("Invalid socket id, not a socket! {}",
                                utils::reflect_name(info.type_).value_or("???"));
  }

  return &(impl_->sockets_[info.real_id_]);
}

Socket* Graph::GetSocket(Pin* output) {
  if (!output) {
    throw make_invalid_argument("Invalid output pin: nullptr");
  }
  return GetSocket(output->socket_in_id_);
}

Socket* Graph::GetSocket(id_t input_pin, id_t output_pin) {
  Socket* sock = GetSocket(GetPin(output_pin));
  if (sock == nullptr) {
    return nullptr;
  }
  AX_DCHECK(sock->input_->id_ == input_pin, "Internal Logic Error!");
  return sock;
}

Socket* Graph::GetSocket(id_t input_node, id_t input_pin, id_t output_node, id_t output_pin) {
  NodeBase* data_in_node = GetNode(input_node);
  NodeBase* data_out_node = GetNode(output_node);
  if (data_in_node == nullptr || data_out_node == nullptr) {
    AX_TRACE("Node not found!");
    return nullptr;
  }

  if (input_pin >= data_in_node->outputs_.size() || output_pin >= data_out_node->inputs_.size()) {
    AX_TRACE("Pin index out of range!");
    return nullptr;
  }

  return GetSocket(data_in_node->outputs_[input_pin].id_, data_out_node->inputs_[output_pin].id_);
}

bool Graph::RemoveSocket(id_t id) { return RemoveSocket(GetSocket(id)); }

bool Graph::RemoveSocket(Socket* sock) {
  if (sock == nullptr) {
    return false;
  }
  Pin* output = sock->output_;
  output->socket_in_id_ = INVALID_ID;
  auto uinfo = impl_->uuid_info_[sock->id_];
  NodeBase* out_node = GetNode(sock->output_->node_id_);
  out_node->inputs_[output->node_io_index_].payload_ = nullptr;
  out_node->CleanUp();
  impl_->RemoveSocket(uinfo.real_id_);
  return sock;
}

bool Graph::RemoveSocket(id_t input_pin, id_t output_pin) {
  Socket* sock = GetSocket(input_pin, output_pin);
  if (sock == nullptr) {
    return false;
  }
  return RemoveSocket(sock);
}

bool Graph::RemoveSocket(id_t input_node, id_t input_pin, id_t output_node, id_t output_pin) {
  Socket* sock = GetSocket(input_node, input_pin, output_node, output_pin);
  if (sock == nullptr) {
    return false;
  }
  return RemoveSocket(sock);
}

Pin* Graph::GetPin(id_t id) {
  auto pin_to_node_info = PinToNode(id);
  if (pin_to_node_info.node_id_ == INVALID_ID || pin_to_node_info.pin_id_ == INVALID_ID) {
    return nullptr;
  }
  NodeBase* node = GetNode(pin_to_node_info.node_id_);
  id_t pi = pin_to_node_info.pin_id_;
  Pin* result = pin_to_node_info.is_input_ ? &(node->inputs_[pi]) : &(node->outputs_[pi]);
  AX_CHECK(result->id_ == id, "{} != {}", result->id_, id);
  return result;
}

Pin const* Graph::GetPin(id_t id) const { return const_cast<Graph*>(this)->GetPin(id); }

PinToNodeInfo Graph::PinToNode(id_t pin_id) const {
  auto it = impl_->pin_to_node_.find(pin_id);
  if (it == impl_->pin_to_node_.end()) {
    return {INVALID_ID, INVALID_ID, true};
  } else {
    NodeBase const* node = GetNode(it->second);
    if (node == nullptr) {
      // node not found, return invalid.
      return {INVALID_ID, INVALID_ID, true};
    }

    for (auto const& pin : node->inputs_) {
      if (pin.id_ == pin_id) {
        return {node->id_, pin.node_io_index_, true};
      }
    }

    for (auto const& pin : node->outputs_) {
      if (pin.id_ == pin_id) {
        return {node->id_, pin.node_io_index_, false};
      }
    }
    AX_CRITICAL("Internal Logic Error!");
    return {node->id_, INVALID_ID, false};
  }
}

void Graph::Clear() {
  for (auto& node : impl_->nodes_) {
    if (node) {
      RemoveNode(node->id_);
    }
  }
  impl_ = std::make_unique<Impl>();
}

id_t Graph::GetNumNodes() const { return (impl_->nodes_.size() - impl_->uuid_reuse_node_.size()); }

id_t Graph::GetNumSockets() const {
  return impl_->sockets_.size() - impl_->uuid_reuse_socket_.size();
}

id_t Graph::GetCurrentUuid() const { return impl_->uuid_next_; }

void Graph::ForeachNode(std::function<void(NodeBase*)> const& func) {
  for (auto& node : impl_->nodes_) {
    if (node) {
      func(node.get());
    }
  }
}

void Graph::ForeachSocket(std::function<void(Socket*)> const& func) {
  for (auto& sock : impl_->sockets_) {
    if (sock.id_ != INVALID_ID) {
      func(&sock);
    }
  }
}

}  // namespace ax::graph
