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


using RenderFn = std::function<void(NodeBase* )>;
struct CustomNodeRender {
  RenderFn widget_;
};

void add_custom_node_render(std::type_index t, CustomNodeRender const& widget);

template <typename Node> void add_custom_node_render(RenderFn const& widget) {
  add_custom_node_render(typeid(Node), CustomNodeRender{widget});
}

CustomNodeRender const* get_custom_node_render(std::type_index t);

void draw_node_content_default(NodeBase* node);

}  // namespace ax::graph