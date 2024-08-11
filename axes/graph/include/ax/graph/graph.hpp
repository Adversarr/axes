#pragma once
#include <functional>

#include "ax/core/common.hpp"
#include "ax/core/config.hpp"
#include "ax/utils/common.hpp"
#include "common.hpp"

namespace ax::graph {

struct PinToNodeInfo {
  ident_t node_id_ = INVALID_ID;
  ident_t pin_id_ = INVALID_ID;
  bool is_input_ = false;

  PinToNodeInfo(ident_t node_id, ident_t pin_id, bool is_input)
      : node_id_(node_id), pin_id_(pin_id), is_input_(is_input) {}

  explicit operator bool() const { return node_id_ != INVALID_ID && pin_id_ != INVALID_ID; }
};

class Graph final {
public:
  // Node management
  NodeBase* AddNode(NodeDescriptor const* descriptor);
  NodeBase* GetNode(ident_t id);
  NodeBase const* GetNode(ident_t id) const;

  bool RemoveNode(ident_t id);
  bool RemoveNode(NodeBase* node);

  // Socket management
  Socket* AddSocket(Pin* input, Pin* output); // TODO: Must return const Socket*
  Socket* AddSocket(ident_t input_pin, ident_t output_pin);
  Socket* AddSocket(ident_t left_node_id, ident_t input_pin, ident_t right_node_id, ident_t output_pin);

  bool CanConnectSocket(Pin const* input, Pin const* output) const;
  bool CanConnectSocket(ident_t input_pin, ident_t output_pin) const;
  bool CanConnectSocket(ident_t input_node, ident_t input_pin, ident_t output_node, ident_t output_pin) const;

  Socket* GetSocket(Pin* output);
  Socket* GetSocket(ident_t socket_id);
  Socket* GetSocket(ident_t input_pin, ident_t output_pin);
  Socket* GetSocket(ident_t input_node, ident_t input_pin, ident_t output_node, ident_t output_pin);

  bool RemoveSocket(Socket* sock);
  bool RemoveSocket(ident_t id);
  bool RemoveSocket(ident_t input_pin, ident_t output_pin);
  bool RemoveSocket(ident_t input_node, ident_t input_pin, ident_t output_node, ident_t output_pin);

  Pin* GetPin(ident_t id);
  Pin const* GetPin(ident_t id) const;
  PinToNodeInfo PinToNode(ident_t pin_id) const;

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

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace ax::graph
