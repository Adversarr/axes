#pragma once
#include "ax/graph/node.hpp"
#include <functional>
namespace ax::graph {

struct GraphRendererOptions {};

/**
 * @brief Install the renderer for the graph editor.
 *
 */
void install_renderer(GraphRendererOptions opt = {});

struct CustomNodeRender {
  std::function<void(NodeBase* )> widget_;
};

void add_custem_node_render(std::type_index t, CustomNodeRender const& widget);

CustomNodeRender const* get_custom_node_render(std::type_index t);

}  // namespace ax::graph