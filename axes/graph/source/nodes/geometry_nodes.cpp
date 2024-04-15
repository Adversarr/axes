#include <imgui_node_editor.h>

#include "ax/geometry/common.hpp"
#include "ax/geometry/normal.hpp"
#include "ax/graph/node.hpp"
#include "ax/graph/render.hpp"
#include "ax/nodes/geometry.hpp"
#include "ax/nodes/gl_prims.hpp"
#include "ax/utils/status.hpp"

using namespace ax::graph;
namespace ed = ax::NodeEditor;

using namespace std;

namespace ax::nodes {

class Normal_PerVertex : public NodeBase {
public:
  Normal_PerVertex(NodeDescriptor const* descriptor, idx id) : NodeBase(descriptor, id) {}

  static void register_this() {
    NodeDescriptorFactory<Normal_PerVertex>()
        .SetName("Normal_PerVertex")
        .SetDescription("Computes per vertex normals")
        .AddInput<geo::SurfaceMesh>("mesh", "The mesh to compute normals for")
        .AddOutput<math::field3r>("normals", "The normals of the mesh")
        .FinalizeAndRegister();
  }

  Status Apply(idx) override {
    auto* mesh = RetriveInput<geo::SurfaceMesh>(0);
    if (mesh == nullptr) {
      return utils::FailedPreconditionError("mesh is not set.");
    }

    math::field3r normals = geo::normal_per_vertex(mesh->vertices_, mesh->indices_);
    SetOutput<math::field3r>(0, std::move(normals));
    AX_RETURN_OK();
  }
};

void register_geometry_nodes() {
  Normal_PerVertex::register_this();
}

}  // namespace ax::nodes
