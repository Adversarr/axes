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

class MakeSurfaceMesh : public NodeBase {
  public:
    MakeSurfaceMesh(NodeDescriptor const* descriptor, idx id) : NodeBase(descriptor, id) {}

    static void register_this() {
      NodeDescriptorFactory<MakeSurfaceMesh>()
          .SetName("Make_SurfaceMesh")
          .SetDescription("Creates a surface mesh")
          .AddInput<math::field3r>("V", "The vertices of the mesh")
          .AddInput<math::field3i>("F", "The faces of the mesh")
          .AddOutput<geo::SurfaceMesh>("mesh", "The resulting surface mesh")
          .FinalizeAndRegister();
    }

    Status Apply(idx) override {
      auto* V = RetriveInput<math::field3r>(0);
      if (V == nullptr) {
        return utils::FailedPreconditionError("V is not set.");
      }
      auto* F = RetriveInput<math::field3i>(1);
      if (F == nullptr) {
        return utils::FailedPreconditionError("F is not set.");
      }
      SetOutput<geo::SurfaceMesh>(0, *V, *F);
      AX_RETURN_OK();
    }
};

class DecomposeSurfaceMesh : public NodeBase {
public:
  DecomposeSurfaceMesh(NodeDescriptor const* descriptor, idx id) : NodeBase(descriptor, id) {}

  static void register_this() {
    NodeDescriptorFactory<DecomposeSurfaceMesh>()
        .SetName("Decompose_SurfaceMesh")
        .SetDescription("Decomposes a surface mesh")
        .AddInput<geo::SurfaceMesh>("mesh", "The mesh to decompose")
        .AddOutput<math::field3r>("V", "The vertices of the mesh")
        .AddOutput<math::field3i>("F", "The faces of the mesh")
        .FinalizeAndRegister();
  }

  Status Apply(idx) override {
    auto* mesh = RetriveInput<geo::SurfaceMesh>(0);
    if (mesh == nullptr) {
      return utils::FailedPreconditionError("mesh is not set.");
    }
    SetOutput<math::field3r>(0, mesh->vertices_);
    SetOutput<math::field3i>(1, mesh->indices_);
    AX_RETURN_OK();
  }
};


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

    add_custom_node_render<Normal_PerVertex>([](NodeBase * node) {
      draw_node_content_default(node);
      auto* n = reinterpret_cast<Normal_PerVertex*>(node);
      ImGui::Text("Strategy: ");
      ImGui::SameLine();
      if (ImGui::Button(n->options[n->selected_option], ImVec2(100, 20))) {
        ImGui::OpenPopup("Normal_PerVertex_Options");
      }

      ed::Suspend();
      if (ImGui::BeginPopup("Normal_PerVertex_Options")) {
        for (int i = 0; i < 3; i++) {
          if (ImGui::Selectable(n->options[i], n->selected_option == i)) {
            n->selected_option = i;
          }
        }
        ImGui::EndPopup();
      }
      ed::Resume();
    });
  }

  Status Apply(idx) override {
    auto* mesh = RetriveInput<geo::SurfaceMesh>(0);
    if (mesh == nullptr) {
      return utils::FailedPreconditionError("mesh is not set.");
    }

    math::field3r normals;
    if (selected_option == 0) {
      normals = geo::normal_per_vertex(mesh->vertices_, mesh->indices_, geo::face_uniform_avg);
    } else if (selected_option == 1) {
      normals = geo::normal_per_vertex(mesh->vertices_, mesh->indices_, geo::face_area_avg);
    } else if (selected_option == 2) {
      normals = geo::normal_per_vertex(mesh->vertices_, mesh->indices_, geo::face_angle_avg);
    }
    SetOutput<math::field3r>(0, std::move(normals));
    AX_RETURN_OK();
  }

  const char * options[3] {
    "uniform",
    "area",
    "angle"
  };
  idx selected_option = 0;
};


void register_geometry_nodes() {
  MakeSurfaceMesh::register_this();
  DecomposeSurfaceMesh::register_this();

  Normal_PerVertex::register_this();
}

}  // namespace ax::nodes
