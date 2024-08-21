/**
 * @file
 * @brief Common definitions for the graph module.
 */

#pragma once
#include <boost/json/object.hpp>

#include "ax/core/config.hpp"

#define CG_NODE_EXTENSION                                      \
  virtual boost::json::object Serialize() const { return {}; } \
  virtual void Deserialize(boost::json::object const &) {}
#include "compute_graph/compute_graph.hpp"

namespace ax::graph {

using namespace ::compute_graph;

using NodePtr = NodeBase *;
using ConstNodePtr = const NodeBase *;
using InputSocketPtr = InputSocket *;
using ConstInputSocketPtr = const InputSocket *;
using OutputSocketPtr = OutputSocket *;
using ConstOutputSocketPtr = const OutputSocket *;

NodeRegistry &get_internal_node_registry();

}  // namespace ax::graph
