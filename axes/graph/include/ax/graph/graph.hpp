#pragma once
#include "ax/core/common.hpp"
#include "ax/core/config.hpp"
#include "common.hpp"

namespace ax::graph {

struct PinToNodeInfo {
  idx node_id_ = invalid_id;
  idx pin_id_ = invalid_id;
  bool is_input_ = false;
};


class Graph final {
public:
  // Node management
  NodeBase* AddNode(NodeDescriptor const* descriptor);
  NodeBase* GetNode(idx id);
  NodeBase const* GetNode(idx id) const;
  bool RemoveNode(idx id);
  bool RemoveNode(NodeBase* node);

  // Socket management
  Socket* AddSocket(Pin* input, Pin* output);
  Socket* AddSocket(idx input_pin, idx output_pin);
  Socket* AddSocket(idx left_node_id, idx input_pin, idx right_node_id, idx output_pin);
  bool CanConnectSocket(Pin const* input, Pin const* output) const;
  bool CanConnectSocket(idx input_pin, idx output_pin) const;
  bool CanConnectSocket(idx input_node, idx input_pin, idx output_node, idx output_pin) const;

  Socket* GetSocket(Pin* output);
  Socket* GetSocket(idx socket_id);
  Socket* GetSocket(idx input_pin, idx output_pin);
  Socket* GetSocket(idx input_node, idx input_pin, idx output_node, idx output_pin);

  bool RemoveSocket(Socket* sock);
  bool RemoveSocket(idx id);
  bool RemoveSocket(idx input_pin, idx output_pin);
  bool RemoveSocket(idx input_node, idx input_pin, idx output_node, idx output_pin);

  Pin* GetPin(idx id);
  Pin const* GetPin(idx id) const;
  PinToNodeInfo PinToNode(idx pin_id) const;

  // Graph management
  void Clear();
  idx GetNumNodes() const;
  idx GetNumSockets() const;
  idx GetCurrentUuid() const;

  std::string Serialize() const;

  // Constructor & Destructor.
  Graph();
  ~Graph();

private:
  struct Impl;
  UPtr<Impl> impl_;
};

}  // namespace ax::graph