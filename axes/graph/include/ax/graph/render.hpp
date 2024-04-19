#pragma once
#include "ax/graph/executor.hpp"
#include "ax/graph/node.hpp"
#include <functional>
namespace ax::graph {

struct GraphRendererOptions {};

UPtr<GraphExecutorBase>& ensure_executor();

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

void draw_node_header_default(NodeBase* node);

void begin_draw_node(NodeBase* node);

void end_draw_node();

}  // namespace ax::graph