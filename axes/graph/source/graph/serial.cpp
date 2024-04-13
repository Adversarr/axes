//
// Created by adversarr on 4/13/24.
//
#include "ax/graph/serial.hpp"

#include "ax/graph/node.hpp"
#include "ax/utils/status.hpp"
using namespace ax;
using namespace ax::graph;

Serializer::Serializer(Graph &g) : graph_(g) {}

void Serializer::SetNodeMetadata(idx node_id, boost::json::value const &obj) {
  node_metadata_[node_id] = obj;
}

void Serializer::SetSocketMetadata(idx socket_id, boost::json::value const &obj) {
  socket_metadata_[socket_id] = obj;
}

boost::json::object Serializer::Serialize() const {
  boost::json::object graph_obj;
  boost::json::array nodes;
  boost::json::array sockets;

  graph_.ForeachNode([&](NodeBase const *n) {
    boost::json::object node_obj;
    node_obj["id"] = n->GetId();
    node_obj["name"] = n->GetDescriptor()->name_;
    if (auto iter = node_metadata_.find(n->GetId()); iter != node_metadata_.end()) {
      node_obj["meta"] = iter->second;
    }
    nodes.push_back(node_obj);
  });

  graph_.ForeachSocket([&](Socket const *s) {
    boost::json::object socket_obj;
    socket_obj["id"] = s->id_;
    socket_obj["input_node"] = s->input_->node_id_;
    socket_obj["input_pin"] = s->input_->node_io_index_;
    socket_obj["output_node"] = s->output_->node_id_;
    socket_obj["output_pin"] = s->output_->node_io_index_;
    if (auto iter = socket_metadata_.find(s->id_); iter != socket_metadata_.end()) {
      socket_obj["meta"] = iter->second;
    }
    sockets.push_back(socket_obj);
  });

  graph_obj["nodes"] = nodes;
  graph_obj["sockets"] = sockets;
  return graph_obj;
}

Deserializer::Deserializer(Graph &g) : graph_(g) {}

Status Deserializer::Deserialize(boost::json::object const &obj) {
  auto nodes = obj.at("nodes").as_array();
  auto sockets = obj.at("sockets").as_array();

  for (auto const &node_obj : nodes) {
    auto id = node_obj.at("id").as_int64();
    auto name = node_obj.at("name").as_string().c_str();
    auto sornode = graph_.AddNode(details::get_node_descriptor(name));
    if (!sornode.ok()) {
      return sornode.status();
    }
    auto node = sornode.value();
    node_id_map_[id] = node->GetId();
    if (auto iter = node_obj.as_object().if_contains("meta");
        iter != nullptr ) {
        node_metadata_[id] = *iter;
    }
  }

  for (auto const &socket_obj : sockets) {
    auto id = socket_obj.at("id").as_int64();
    auto input_node = socket_obj.at("input_node").as_int64();
    auto input_pin = socket_obj.at("input_pin").as_int64();
    auto output_node = socket_obj.at("output_node").as_int64();
    auto output_pin = socket_obj.at("output_pin").as_int64();
    auto in_node = node_id_map_[input_node];
    auto out_node = node_id_map_[output_node];
    auto const input = graph_.GetNode(in_node);
    auto const output = graph_.GetNode(out_node);
    auto socket = graph_.AddSocket(input->GetId(), input_pin,
                                   output->GetId(), output_pin);
    if (!socket.ok()) {
      return socket.status();
    }
    socket_id_map_[id] = socket.value()->id_;
    if (auto iter = socket_obj.as_object().if_contains("meta");
        iter != nullptr) {
        socket_metadata_[id] = *iter;
    }
  }
  AX_RETURN_OK();
}