#include "ax/graph/serial.hpp"

#include <fmt/ostream.h>

#include <range/v3/view/enumerate.hpp>
#include <range/v3/view/filter.hpp>
#include <range/v3/view/indirect.hpp>

#include "ax/core/logging.hpp"
using namespace ax;
using namespace ax::graph;
using namespace ranges::views;
using namespace boost::json;


Serializer::Serializer(Graph &g) : graph_(g) {}

void Serializer::SetNodeMetadata(ConstNodePtr n, object const &obj) { node_metadata_[n] = obj; }

object Serializer::Serialize() const {
  array nodes, pipes, strong_link;
  for (const auto &[i, n] : enumerate(graph_.GetNodes())) {
    object node{{"index", i}, {"name", n->GetDescriptor().Name()}};

    if (auto it = node_metadata_.find(n.get()); it != node_metadata_.end()) {
      node["meta"] = it->second;
    } else {
      node["meta"] = object{};
    }

    for (const auto &in_sock : n->GetInputs() | filter([](const auto &s) { return s.IsConnected(); })) {
      auto from_node_handle = graph_.Get(&(in_sock.From()->Node()));
      AX_CHECK(from_node_handle, "Failed to get node handle from socket: {}", fmt::ptr(in_sock.From()));
      object socket_obj{{"to", i},
                        {"to_socket_index", in_sock.Index()},
                        {"from", from_node_handle->Index()},
                        {"from_socket_index", in_sock.From()->Index()}};
      pipes.emplace_back(std::move(socket_obj));
    }

    for (const NodeBase *pred : n->GetPrecedents()) {
      auto from_node_handle = graph_.Get(pred);
      AX_CHECK(from_node_handle, "Failed to get node handle from socket: {}", fmt::ptr(pred));
      object link{{"from", i}, {"to", from_node_handle->Index()}};

      strong_link.emplace_back(std::move(link));
    }

    nodes.emplace_back(std::move(node));
  }

  return object{{"nodes", std::move(nodes)}, {"pipes", std::move(pipes)}, {"strong_link", std::move(strong_link)}};
}

void Serializer::Deserialize(object const &obj) {
  graph_.Clear();
  node_metadata_.clear();
  array nodes = obj.at("nodes").as_array();
  array pipes = obj.at("pipes").as_array();
  array strong_link = obj.at("strong_link").as_array();
  auto &registry = NodeRegistry::instance();

  size_t num_nodes = nodes.size();
  std::vector<bool> available_nodes(num_nodes, false);
  std::vector<size_t> map_to_new_index(num_nodes, 0);

  for (value node_info : nodes) {
    if (! node_info.is_object()) {
      AX_ERROR("Node is not an object: {}", fmt::streamed(node_info));
    }
    std::string name = node_info.at("name").as_string().c_str();
    size_t index = node_info.at("index").as_int64();

    auto node = registry.Create(name);
    if (!node) {
      AX_ERROR("At {}, failed to create node: {}", index, name);
      continue;
    }

    // Node is created, push to graph.
    auto handle = graph_.PushBack(std::move(node));
    available_nodes[index] = true;
    map_to_new_index[index] = handle.Index();
    if (const value *iter = node_info.as_object().if_contains("meta")) {
      if (!iter->is_object()) {
        AX_ERROR("At {}, meta is not an object", index);
      } else {
        node_metadata_[handle.operator->()] = iter->as_object();
      }
    }
  }

  for (value pipe : pipes) {
    if (!pipe.is_object()) {
      AX_ERROR("Pipe is not an object: {}", fmt::streamed(pipe));
    }

    auto obj = pipe.as_object();
    size_t to = obj.at("to").as_int64(), from = obj.at("from").as_int64(),
           to_socket_index = obj.at("to_socket_index").as_int64(),
           from_socket_index = obj.at("from_socket_index").as_int64();

    if (available_nodes[to] && available_nodes[from]) {
      auto from_node = graph_[map_to_new_index[from]];
      auto to_node = graph_[map_to_new_index[to]];
      auto from_sock = from_node.GetOutput(from_socket_index);
      auto to_sock = to_node.GetInput(to_socket_index);
      if (can_connect(*from_sock, *to_sock)) {
        graph_.Connect(from_sock, to_sock);
      } else {
        AX_ERROR("socket I [{}:{}] O [{}:{}] cannot be connected!", to, to_socket_index, from, from_socket_index);
      }
    } else {
      AX_WARN("socket I [{}:{}] O [{}:{}] will not be connected", to, to_socket_index, from, from_socket_index);
    }
  }

  for (value link : strong_link) {
    if (!link.is_object()) {
      AX_ERROR("Link is not an object: {}", fmt::streamed(link));
    }

    auto obj = link.as_object();
    size_t from = obj.at("from").as_int64(), to = obj.at("to").as_int64();

    if (available_nodes[to] && available_nodes[from]) {
      auto from_node = graph_[map_to_new_index[from]];
      auto to_node = graph_[map_to_new_index[to]];
      graph_.Connect(from_node, to_node);
    } else {
      AX_WARN("Link I [{}] O [{}] will not be connected", to, from);
    }
  }
}
