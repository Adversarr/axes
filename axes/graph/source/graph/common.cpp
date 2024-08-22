//
// Created by adversarr on 2024/8/18.
//
#include "ax/graph/common.hpp"

#include "../nodes/all.hpp"
#include "ax/core/entt.hpp"

namespace ax::graph {
using namespace nodes;

static void register_all_internal_nodes(NodeRegistry& reg) {
  register_stl_types(reg);
  register_io(reg);
  register_math_types(reg);
  register_gl(reg);
}

NodeRegistry& get_internal_node_registry() {
  auto& reg = ensure_resource<NodeRegistry>();
  static std::once_flag flag;
  std::call_once(flag, [&reg]() { register_all_internal_nodes(reg); });
  return reg;
}

}  // namespace ax::graph