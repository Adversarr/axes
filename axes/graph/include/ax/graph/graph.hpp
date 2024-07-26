#pragma once
#include "ax/core/common.hpp"
#include "ax/core/config.hpp"
#include "ax/utils/common.hpp"
#include "common.hpp"

namespace ax::graph {

struct PinToNodeInfo {
  idx node_id_ = INVALID_ID;
  idx pin_id_ = INVALID_ID;
  bool is_input_ = false;
};


class Graph final {
public:
  // Node management
  StatusOr<NodeBase*> AddNode(NodeDescriptor const* descriptor);
  NodeBase* GetNode(idx id);
  NodeBase const* GetNode(idx id) const;

  bool RemoveNode(idx id);
  bool RemoveNode(NodeBase* node);

  // Socket management
  StatusOr<Socket*> AddSocket(Pin* input, Pin* output);
  StatusOr<Socket*> AddSocket(idx input_pin, idx output_pin);
  StatusOr<Socket*> AddSocket(idx left_node_id, idx input_pin, idx right_node_id, idx output_pin);

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

  // Constructor & Destructor.
  Graph();
  AX_DECLARE_CONSTRUCTOR(Graph, delete, delete);
  ~Graph();

  void ForeachNode(std::function<void(NodeBase*)> const& func);
  void ForeachSocket(std::function<void(Socket*)> const& func);
  void EnsurePayloads();
private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace ax::graph
