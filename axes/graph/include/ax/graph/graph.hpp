#pragma once
#include <functional>

#include "ax/core/common.hpp"
#include "ax/core/config.hpp"
#include "ax/utils/common.hpp"
#include "common.hpp"

namespace ax::graph {

struct PinToNodeInfo {
  id_t node_id_ = INVALID_ID;
  id_t pin_id_ = INVALID_ID;
  bool is_input_ = false;

  PinToNodeInfo(id_t node_id, id_t pin_id, bool is_input)
      : node_id_(node_id), pin_id_(pin_id), is_input_(is_input) {}

  explicit operator bool() const { return node_id_ != INVALID_ID && pin_id_ != INVALID_ID; }
};

class Graph final {
public:
  // Node management
  NodeBase* AddNode(NodeDescriptor const* descriptor);
  NodeBase* GetNode(id_t id);
  NodeBase const* GetNode(id_t id) const;

  bool RemoveNode(id_t id);
  bool RemoveNode(NodeBase* node);

  // Socket management
  Socket* AddSocket(Pin* input, Pin* output);
  Socket* AddSocket(id_t input_pin, id_t output_pin);
  Socket* AddSocket(id_t left_node_id, id_t input_pin, id_t right_node_id, id_t output_pin);

  bool CanConnectSocket(Pin const* input, Pin const* output) const;
  bool CanConnectSocket(id_t input_pin, id_t output_pin) const;
  bool CanConnectSocket(id_t input_node, id_t input_pin, id_t output_node, id_t output_pin) const;

  Socket* GetSocket(Pin* output);
  Socket* GetSocket(id_t socket_id);
  Socket* GetSocket(id_t input_pin, id_t output_pin);
  Socket* GetSocket(id_t input_node, id_t input_pin, id_t output_node, id_t output_pin);

  bool RemoveSocket(Socket* sock);
  bool RemoveSocket(id_t id);
  bool RemoveSocket(id_t input_pin, id_t output_pin);
  bool RemoveSocket(id_t input_node, id_t input_pin, id_t output_node, id_t output_pin);

  Pin* GetPin(id_t id);
  Pin const* GetPin(id_t id) const;
  PinToNodeInfo PinToNode(id_t pin_id) const;

  // Graph management
  void Clear();
  size_t GetNumNodes() const;
  size_t GetNumSockets() const;
  size_t GetCurrentUuid() const;

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
