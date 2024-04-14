#include <imgui.h>
#include <imgui_node_editor.h>

#include "ax/core/entt.hpp"
#include "ax/core/init.hpp"
#include "ax/gl/context.hpp"
#include "ax/gl/utils.hpp"
#include "ax/graph/graph.hpp"
#include "ax/graph/node.hpp"
#include "ax/graph/render.hpp"
#include "ax/nodes/gl_prims.hpp"
#include "ax/nodes/io.hpp"
#include "ax/nodes/math_types.hpp"
#include "ax/nodes/stl_types.hpp"
#include "ax/utils/asset.hpp"
#include "ax/utils/status.hpp"

using namespace ax;
using namespace ax::graph;

int main(int argc, char** argv) {
  gl::init(argc, argv);
  graph::install_renderer();
  nodes::register_stl_types();
  nodes::register_io_nodes();
  nodes::register_math_types_nodes();
  nodes::register_gl_prim_nodes();

  AX_CHECK_OK(gl::enter_main_loop());
  clean_up();
  return 0;
}
