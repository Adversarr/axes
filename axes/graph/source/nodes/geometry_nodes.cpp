#include <imgui_node_editor.h>

#include "ax/geometry/common.hpp"
#include "ax/geometry/normal.hpp"
#include "ax/geometry/primitives.hpp"
#include "ax/geometry/topology.hpp"
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
          .AddInput<math::field3r>("vertices", "The vertices of the mesh")
          .AddInput<math::field3i>("faces", "The faces of the mesh")
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
        .AddOutput<math::field3r>("vertices", "The vertices of the mesh")
        .AddOutput<math::field3i>("faces", "The faces of the mesh")
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
      begin_draw_node(node);
      draw_node_header_default(node);
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
      end_draw_node();
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

class Make_XyPlane : public NodeBase {
public:
  Make_XyPlane(NodeDescriptor const* descriptor, idx id) : NodeBase(descriptor, id) {}

  static void register_this() {
    NodeDescriptorFactory<Make_XyPlane>()
        .SetName("Make_XyPlane")
        .SetDescription("Creates a xy plane (Default = [0,1]x[0,1] with 1x1 resolution)")
        .AddInput<math::vec2r>("size", "The size of the plane")
        .AddInput<math::vec2i>("resolution", "The resolution of the plane")
        .AddOutput<geo::SurfaceMesh>("mesh", "The resulting surface mesh")
        .FinalizeAndRegister();
  }

  Status Apply(idx) override {
    auto size = RetriveInput<math::vec2r>(0);
    auto resolution = RetriveInput<math::vec2i>(1);
    math::vec2r size_inuse = size ? *size : math::vec2r(1, 1);
    math::vec2i resolution_inuse = resolution ? *resolution : math::vec2i(1, 1);
    if (resolution_inuse.x() < 1 || resolution_inuse.y() < 1) {
      return utils::FailedPreconditionError("Resolution must be at least 1x1.");
    }
    auto mesh = geo::plane(size_inuse.x(), size_inuse.y(), resolution_inuse.x(), resolution_inuse.y());
    SetOutput<geo::SurfaceMesh>(0, std::move(mesh));
    AX_RETURN_OK();
  }
};

class ExtractBoundary_Tetrahedrons : public NodeBase {
public:
  ExtractBoundary_Tetrahedrons(NodeDescriptor const* descriptor, idx id) : NodeBase(descriptor, id) {}

  static void register_this() {
    NodeDescriptorFactory<ExtractBoundary_Tetrahedrons>()
        .SetName("ExtractBoundary_Tetrahedrons_pre")
        .SetDescription("Extracts the boundary of a tetrahedral mesh")
        .AddInput<math::field3r>("vertices", "The vertices of the mesh")
        .AddInput<math::field4i>("tetras", "The tetrahedrons of the mesh")
        .AddInput<bool>("reload", "Whether to recompute the boundary every frame")
        .AddOutput<math::field3i>("bd_faces", "The boundary of the mesh")
        .FinalizeAndRegister();
  }

  Status PreApply() override {
    auto* V = RetriveInput<math::field3r>(0);
    auto* T = RetriveInput<math::field4i>(1);
    if (V == nullptr) {
      return utils::FailedPreconditionError("V is not set.");
    }

    if (T == nullptr) {
      return utils::FailedPreconditionError("T is not set.");
    }

    if (V->cols() == 0 || T->cols() == 0) {
      AX_RETURN_OK();
    }

    auto boundary = geo::get_boundary_triangles(*V, *T);
    SetOutput<math::field3i>(0, std::move(boundary));
    AX_RETURN_OK();
  }

  Status Apply(idx frame) final {
    if (auto *reload = RetriveInput<bool>(2); (frame == 0) || (reload && *reload)) {
      return PreApply();
    }
    AX_RETURN_OK();
  }
};

void register_geometry_nodes() {
  MakeSurfaceMesh::register_this();
  DecomposeSurfaceMesh::register_this();

  Normal_PerVertex::register_this();
  Make_XyPlane::register_this();
  ExtractBoundary_Tetrahedrons::register_this();
}

}  // namespace ax::nodes
