#include "ax/graph/graph.hpp"

#include <absl/cleanup/cleanup.h>

#include <boost/describe/enum.hpp>
#include <iomanip>
#include <map>

#include "ax/core/echo.hpp"
#include "ax/graph/node.hpp"

#if WIN32
#  undef ERROR
#endif

namespace ax::graph {

BOOST_DEFINE_ENUM(UuidInferType, kNode, kPin, kSocket, kInvalid);

struct UuidUnderlyingInfo {
  UuidInferType const type_;
  idx real_id_;
};

struct Graph::Impl {
  // Uuid.
  List<idx> uuid_reuse_node_, uuid_reuse_pin_, uuid_reuse_socket_;
  idx uuid_next_ = 0;
  List<UuidUnderlyingInfo> uuid_info_;
  std::map<idx, idx> pin_to_node_;
  std::map<idx, std::set<idx>> node_to_sockets_;

  // Container for graph elements.
  List<UPtr<NodeBase>> nodes_;
  List<Socket> sockets_;
  List<Payload> payloads_;

  Impl() {
    uuid_info_.emplace_back(UuidUnderlyingInfo{kInvalid, INVALID_ID});
    uuid_next_ = 1;
  }

  idx UuidStep(UuidInferType underlying) {
    idx id = 0;
    switch (underlying) {
      case kNode:
        if (uuid_reuse_node_.empty()) {
          id = uuid_next_++;
        } else {
          id = uuid_reuse_node_.back();
          uuid_reuse_node_.pop_back();
        }
        break;
      case kPin:
        if (uuid_reuse_pin_.empty()) {
          id = uuid_next_++;
        } else {
          id = uuid_reuse_pin_.back();
          uuid_reuse_pin_.pop_back();
        }
        break;
      case kSocket:
        if (uuid_reuse_socket_.empty()) {
          id = uuid_next_++;
        } else {
          id = uuid_reuse_socket_.back();
          uuid_reuse_socket_.pop_back();
        }
        break;
      default:
        AX_DCHECK(underlying != underlying) << "Internal Logic Error!";
    }

    if (id >= (idx)uuid_info_.size()) {
      uuid_info_.emplace_back(UuidUnderlyingInfo{underlying, INVALID_ID});
    }
    return id;
  }

  void EmplaceNode(UPtr<NodeBase> n) {
    AX_DCHECK(n->id_ <= (idx)uuid_info_.size()) << "Internal Logic Error!";
    auto& uinfo = uuid_info_[n->id_];
    node_to_sockets_.insert({n->id_, std::set<idx>{}});
    if (uinfo.real_id_ == INVALID_ID) {
      uinfo.real_id_ = (idx)nodes_.size();
      nodes_.push_back(std::move(n));
    } else {
      AX_DCHECK(nodes_[uinfo.real_id_].get() == nullptr)
          << "You are trying to overwrite an existing node!";
      nodes_[uinfo.real_id_] = std::move(n);
    }

    for (auto& pin : nodes_[uinfo.real_id_]->inputs_) {
      pin_to_node_[pin.id_] = nodes_[uinfo.real_id_]->id_;
    }
    for (auto& pin : nodes_[uinfo.real_id_]->outputs_) {
      pin_to_node_[pin.id_] = nodes_[uinfo.real_id_]->id_;
    }
  }

  void RemoveNode(idx i) {
    auto& n = nodes_[i];
    idx uid = n->id_;
    n.reset(nullptr);
    UuidRecycle(uid);
  }

  void EmplaceSocket(Socket s) {
    AX_DCHECK(s.id_ <= (idx)uuid_info_.size()) << "Internal Logic Error!";
    auto& uinfo = uuid_info_[s.id_];
    if (uinfo.real_id_ == INVALID_ID) {
      uinfo.real_id_ = (idx)sockets_.size();
      sockets_.push_back(s);
    } else {
      AX_DCHECK(sockets_[uinfo.real_id_].id_ == INVALID_ID)
          << "You are trying to overwrite an existing socket!";
      sockets_[uinfo.real_id_] = s;
    }
    node_to_sockets_[s.input_->node_id_].insert(s.id_);
    node_to_sockets_[s.output_->node_id_].insert(s.id_);
  }

  void RemoveSocket(idx i) {
    auto& s = sockets_[i];
    auto uid = s.id_;
    node_to_sockets_[s.input_->node_id_].erase(uid);
    node_to_sockets_[s.output_->node_id_].erase(uid);
    s.id_ = INVALID_ID;
    AX_DLOG(INFO) << "Socket <uid=" << i << "> removed.";
    UuidRecycle(uid);
  }

  void UuidRecycle(idx id) {
    auto& info = uuid_info_[id];
    AX_DLOG(INFO) << "Reusing " << id << ", type: " << (idx)info.type_;
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
        AX_DCHECK(info.type_ != info.type_) << "Internal Logic Error!";
    }
  }
};

Graph::Graph() { impl_ = std::make_unique<Impl>(); }
Graph::~Graph() { Clear(); }

StatusOr<NodeBase*> Graph::AddNode(NodeDescriptor const* descriptor) {
  idx id = impl_->UuidStep(kNode);
  auto n = NodeBase::Create(descriptor, id);
  absl::InlinedVector<idx, 32> ids_used;
  bool build_failed = false;

  absl::Cleanup clup = [&ids_used, &build_failed, this]() {
    if (build_failed) {
      for (auto id : ids_used) {
        impl_->UuidRecycle(id);
      }
    }
  };

  // Not found.
  if_unlikely(!n) {
    build_failed = true;
    AX_LOG(ERROR) << "Node " << descriptor->name_ << " not found.";
    return nullptr;
  }

  // Construct the node Input and Output.
  idx count = 0;
  n->inputs_.reserve(descriptor->inputs_.size());
  n->outputs_.reserve(descriptor->outputs_.size());
  n->output_payloads_.reserve(descriptor->outputs_.size());

  for (auto const& p_in_desc : descriptor->inputs_) {
    idx uid = impl_->UuidStep(kPin);
    ids_used.push_back(uid);
    AX_DLOG(INFO) << "Input Pin " << std::quoted(p_in_desc.name_) << " added. <uid=" << uid << ">";
    n->inputs_.emplace_back(Pin{n->id_, count++, true, &p_in_desc, uid, nullptr});
  }
  count = 0;
  for (auto const& p_out_desc : descriptor->outputs_) {
    idx uid = impl_->UuidStep(kPin);
    ids_used.push_back(uid);
    AX_DLOG(INFO) << "Output Pin " << std::quoted(p_out_desc.name_) << " added. <uid=" << uid
                  << ">";
    n->output_payloads_.emplace_back(Payload::Create(p_out_desc.type_));
    n->outputs_.emplace_back(Pin{n->id_, count++, false, &p_out_desc, uid, &n->output_payloads_.back()});
  }


  // Call the OnConstruct function.
  if (auto status = n->OnConstruct(); !status.ok()) {
    build_failed = true;
    return status;
  }

  // Everything is ok.
  NodeBase* node = n.get();
  impl_->EmplaceNode(std::move(n));
  AX_DLOG(INFO) << "Node " << std::quoted(descriptor->name_) << " added. <uid=" << node->id_ << ">";
  return node;
}

NodeBase* Graph::GetNode(idx id) {
  auto const& uinfo = impl_->uuid_info_;
  if (id >= (idx)uinfo.size()) {
    AX_LOG(WARNING) << "Node <uid=" << id << "> not found. (out of range)";
    return nullptr;
  }

  auto& info = uinfo[id];
  if (info.type_ != kNode) {
    AX_LOG(WARNING) << "Node <uid=" << id << "> not found. (uuid type not node)";
    return nullptr;
  }
  AX_DCHECK(info.real_id_ < (idx)impl_->nodes_.size()) << "Internal Logic Error!";
  return impl_->nodes_[info.real_id_].get();
}

NodeBase const* Graph::GetNode(idx id) const { return const_cast<Graph*>(this)->GetNode(id); }

bool Graph::RemoveNode(NodeBase* n) { return RemoveNode(n->id_); }

bool Graph::RemoveNode(idx id) {
  auto const& uinfo = impl_->uuid_info_;
  if (id >= (idx)uinfo.size()) {
    return false;
  }

  auto& info = uinfo[id];
  if (info.type_ != kNode) {
    return false;
  }

  auto node = impl_->nodes_[info.real_id_].get();
  for (auto& pin : node->inputs_) {
    impl_->UuidRecycle(pin.id_);
  }
  for (auto& pin : node->outputs_) {
    impl_->UuidRecycle(pin.id_);
  }
  node->CleanUp();
  node->OnDestroy();
  if (auto it = impl_->node_to_sockets_.find(id); it != impl_->node_to_sockets_.end()) {
    auto copy = it->second;
    for (idx sock_id : copy) {
      RemoveSocket(sock_id);
    }
    impl_->node_to_sockets_.erase(id);
  }
  impl_->RemoveNode(info.real_id_);
  AX_DLOG(INFO) << "Node <uid=" << id << "> removed.";
  return true;
}

StatusOr<Socket*> Graph::AddSocket(Pin* input, Pin* output) {
  if (!input || !output) {
    return nullptr;
  }
  if (!CanConnectSocket(input, output)) {
    return nullptr;
  }
  if (auto existing_socket = GetSocket(output); existing_socket) {
    AX_DLOG(INFO) << "Socket on " << output->node_id_
                 << " already exists! Remove before add a new one";
    RemoveSocket(existing_socket);
  }

  idx sock_id = impl_->UuidStep(kSocket);
  Socket socket{sock_id, input, output};

  NodeBase* in_node = GetNode(input->node_id_);
  NodeBase* out_node = GetNode(output->node_id_);

  socket.output_->payload_ = &in_node->output_payloads_[input->node_io_index_];

  // Set the input pin to the socket.
  impl_->EmplaceSocket(socket);
  output->socket_in_id_ = sock_id;
  AX_DLOG(INFO) << "Socket " << sock_id << " added. <" << input->id_ << "--" << output->id_ << ">";
  if (auto s = out_node->OnConnect(socket.output_->node_io_index_); !s.ok()) {
    return s;
  }
  return &(impl_->sockets_[impl_->uuid_info_[sock_id].real_id_]);
}

StatusOr<Socket*> Graph::AddSocket(idx input_pin, idx output_pin) {
  Pin *input = GetPin(input_pin), *output = GetPin(output_pin);
  if (!input || !output) {
    return nullptr;
  }
  return AddSocket(input, output);
}

StatusOr<Socket*> Graph::AddSocket(idx left, idx input_pin, idx right, idx output_pin) {
  NodeBase* data_in_node = GetNode(left);
  NodeBase* data_out_node = GetNode(right);
  // The node does not exist
  if (!data_in_node || !data_out_node) {
    AX_LOG(ERROR) << "Node not found!";
    return nullptr;
  }

  if (input_pin >= (idx)data_in_node->outputs_.size()
      || output_pin >= (idx)data_out_node->inputs_.size()) {
    AX_LOG(ERROR) << "Pin index out of range!";
    return nullptr;
  }

  return AddSocket(&(data_in_node->outputs_[input_pin]), &(data_out_node->inputs_[output_pin]));
}

bool Graph::CanConnectSocket(Pin const* input, Pin const* output) const {
  if (!input || !output) {
    return false;
  }
  if (input->is_input_ || !output->is_input_) {
    AX_LOG(WARNING) << "Link is only available when O->I";
    return false;
  }
  if (input->descriptor_->type_ != output->descriptor_->type_) {
    AX_LOG(WARNING) << "Link is only available when the type is the same"
                    << input->descriptor_->type_.name()
                    << " != " << output->descriptor_->type_.name();
    return false;
  }
  return true;
}

bool Graph::CanConnectSocket(idx input_pin, idx output_pin) const {
  Pin const *input = GetPin(input_pin), *output = GetPin(output_pin);
  return CanConnectSocket(input, output);
}

bool Graph::CanConnectSocket(idx input_node, idx input_pin, idx output_node, idx output_pin) const {
  NodeBase const* data_in_node = GetNode(input_node);
  NodeBase const* data_out_node = GetNode(output_node);
  if (data_in_node == nullptr || data_out_node == nullptr) {
    AX_LOG(ERROR) << "Node not found!";
    return false;
  }

  if (input_pin >= (idx)data_in_node->outputs_.size()) {
    AX_LOG(ERROR) << "Pin index out of range!";
    return false;
  }
  if (output_pin >= (idx)data_out_node->inputs_.size()) {
    AX_LOG(ERROR) << "Pin index out of range!";
    return false;
  }

  return CanConnectSocket(&(data_in_node->outputs_[input_pin]),
                          &(data_out_node->inputs_[output_pin]));
}

Socket* Graph::GetSocket(idx id) {
  auto const& uinfo = impl_->uuid_info_;
  if (id >= (idx)uinfo.size() || id < 0) {
    return nullptr;
  }

  auto& info = uinfo[id];
  if (info.type_ != kSocket) {
    return nullptr;
  }

  return &(impl_->sockets_[info.real_id_]);
}

Socket* Graph::GetSocket(Pin* output) {
  if (!output) {
    return nullptr;
  }
  return GetSocket(output->socket_in_id_);
}

Socket* Graph::GetSocket(idx input_pin, idx output_pin) {
  auto sock = GetSocket(GetPin(output_pin));
  if (sock == nullptr) {
    return nullptr;
  }
  AX_DCHECK(sock->input_->id_ == input_pin) << "Internal Logic Error!";
  return sock;
}

Socket* Graph::GetSocket(idx input_node, idx input_pin, idx output_node, idx output_pin) {
  NodeBase* data_in_node = GetNode(input_node);
  NodeBase* data_out_node = GetNode(output_node);
  if (data_in_node == nullptr || data_out_node == nullptr) {
    AX_DLOG(INFO) << "Node not found!";
    return nullptr;
  }

  if (input_pin >= (idx)data_in_node->outputs_.size()
      || output_pin >= (idx)data_out_node->inputs_.size()) {
    AX_DLOG(INFO) << "Pin index out of range!";
    return nullptr;
  }

  return GetSocket(data_in_node->outputs_[input_pin].id_, data_out_node->inputs_[output_pin].id_);
}

bool Graph::RemoveSocket(idx id) { return RemoveSocket(GetSocket(id)); }

bool Graph::RemoveSocket(Socket* sock) {
  if (sock == nullptr) {
    return false;
  }
  auto output = sock->output_;
  output->socket_in_id_ = INVALID_ID;
  auto uinfo = impl_->uuid_info_[sock->id_];
  NodeBase* out_node = GetNode(sock->output_->node_id_);
  out_node->inputs_[output->node_io_index_].payload_ = nullptr;
  out_node->CleanUp();
  impl_->RemoveSocket(uinfo.real_id_);
  return sock;
}

bool Graph::RemoveSocket(idx input_pin, idx output_pin) {
  auto sock = GetSocket(input_pin, output_pin);
  if (sock == nullptr) {
    return false;
  }
  return RemoveSocket(sock);
}

bool Graph::RemoveSocket(idx input_node, idx input_pin, idx output_node, idx output_pin) {
  auto sock = GetSocket(input_node, input_pin, output_node, output_pin);
  if (sock == nullptr) {
    return false;
  }
  return RemoveSocket(sock);
}

Pin* Graph::GetPin(idx id) {
  auto pin_to_node_info = PinToNode(id);
  if (pin_to_node_info.node_id_ == INVALID_ID || pin_to_node_info.pin_id_ == INVALID_ID) {
    return nullptr;
  }
  NodeBase* node = GetNode(pin_to_node_info.node_id_);
  idx pi = pin_to_node_info.pin_id_;
  Pin* result = pin_to_node_info.is_input_ ? &(node->inputs_[pi]) : &(node->outputs_[pi]);
  AX_CHECK(result->id_ == id) << result->id_ << " != " << id;
  return result;
}

Pin const* Graph::GetPin(idx id) const { return const_cast<Graph*>(this)->GetPin(id); }

PinToNodeInfo Graph::PinToNode(idx pin_id) const {
  auto it = impl_->pin_to_node_.find(pin_id);
  if (it == impl_->pin_to_node_.end()) {
    return {INVALID_ID, INVALID_ID, true};
  } else {
    NodeBase const* node = GetNode(it->second);
    if (node == nullptr) {
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
    AX_DLOG(ERROR) << "Internal Logic Error!";
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

idx Graph::GetNumNodes() const {
  return (idx)(impl_->nodes_.size() - impl_->uuid_reuse_node_.size());
}

idx Graph::GetNumSockets() const {
  return (idx)impl_->sockets_.size() - impl_->uuid_reuse_socket_.size();
}

idx Graph::GetCurrentUuid() const { return impl_->uuid_next_; }

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

void Graph::EnsurePayloads() {
}

}  // namespace ax::graph
