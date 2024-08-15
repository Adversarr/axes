#pragma once
#include <functional>

#include "ax/graph/executor.hpp"
namespace ax::graph {

struct GraphRendererOptions {};

std::unique_ptr<GraphExecutorBase>& ensure_executor();

/**
 * @brief Install the renderer for the graph editor.
 *
 */
void install_renderer(GraphRendererOptions opt = {});

using RenderFn = std::function<void(NodeBase*)>;
struct CustomNodeRender {
  RenderFn widget_;
};

void add_custom_node_render(std::string t, CustomNodeRender const& widget);
CustomNodeRender const* get_custom_node_render(std::string name);

void begin_draw_node(NodeBase* node);
void draw_node_header_default(NodeBase* node);
void draw_node_content_default(NodeBase* node);
void end_draw_node();

}  // namespace ax::graph
