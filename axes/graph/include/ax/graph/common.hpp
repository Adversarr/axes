/**
 * @file
 * @brief Common definitions for the graph module.
 */

#pragma once

#include <typeindex>

#include "ax/core/config.hpp"
namespace ax::graph {

using std::type_index; ///< @brief The type index, identify the type

// We create Pin, Node from descriptors.
struct PinDescriptor;
struct NodeDescriptor;

// NOTE: The id_t is used to identify the unique id of a node, pin, or socket.
using id_t = size_t;

// NOTE: A payload is an alternative for std::any to store the actual data of a Pin.
class Payload;

// NOTE: A pin is a connection point of a node, it can be an input or an output.
//       The pin does not own the memory of the payload. It just holds a pointer to the payload.
struct Pin;

// NOTE:
//   +-----------+
//   |  NodeBase |
//   +-----------+
//  <+-I   |   O-+>
//  <+-I   |   O-+>
//  <+-I   |   O-+>
//   +-----------+
//   Each I/O is a Pin.
//   - Each Input Pin **can only connect to one socket atmost**.
//   - Each Output Pin **can connect to multiple sockets**.
//   Node itself does not control the allocation of memory of its input pins, but for outputs,
//   its allocation is automatically performed after the node is created.
class NodeBase;

// NOTE: Graph, the manager of all the Nodes, Pins, Sockets
class Graph;

constexpr id_t INVALID_ID = 0xFFFFFFFF;

// NOTE:
//   Each Socket connects an input Pin to an output Pin, this concept is quite similar to the
//   concept of a wire in a circuit.
struct Socket {
  Socket(id_t id, Pin* input, Pin* out) : input_(input), output_(out), id_(id) {}
  Pin* input_;
  Pin* output_;
  id_t id_;
};

}  // namespace ax::graph
