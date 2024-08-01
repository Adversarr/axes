#include <imgui_node_editor.h>

#include "ax/core/excepts.hpp"
#include "ax/geometry/common.hpp"
#include "ax/geometry/normal.hpp"
#include "ax/geometry/primitives.hpp"
#include "ax/geometry/topology.hpp"
#include "ax/graph/node.hpp"
#include "ax/graph/render.hpp"
#include "ax/nodes/geometry.hpp"
#include "ax/nodes/gl_prims.hpp"

using namespace ax::graph;
namespace ed = ax::NodeEditor;

using namespace std;

namespace ax::nodes {

class MakeSurfaceMesh : public NodeBase {
public:
  AX_NODE_COMMON_WITH_CTOR(MakeSurfaceMesh, "Make_SurfaceMesh", "Creates a surface mesh");
  AX_NODE_INPUTS((math::field3r, vertices, "The vertices of the mesh"),
                 (math::field3i, faces, "The faces of the mesh"));
  AX_NODE_OUTPUTS((geo::SurfaceMesh, mesh, "The resulting surface mesh"));

  void Apply(size_t) override {
    const auto& v = AX_NODE_INPUT_ENSURE_EXTRACT(vertices);
    const auto& f = AX_NODE_INPUT_ENSURE_EXTRACT(faces);
    SetOutput<geo::SurfaceMesh>(0, v, f);
  }
};

class DecomposeSurfaceMesh : public NodeBase {
public:
  AX_NODE_COMMON_WITH_CTOR(DecomposeSurfaceMesh, "Decompose_SurfaceMesh",
                           "Decomposes a surface mesh");
  AX_NODE_INPUTS((geo::SurfaceMesh, mesh, "The mesh to decompose"));
  AX_NODE_OUTPUTS((math::field3r, vertices, "The vertices of the mesh"),
                  (math::field3i, faces, "The faces of the mesh"));

  void Apply(size_t) override {
    auto const& mesh = AX_NODE_INPUT_ENSURE_EXTRACT(mesh);
    SetOutput<math::field3r>(0, mesh.vertices_);
    SetOutput<math::field3i>(1, mesh.indices_);
  }
};

class Normal_PerVertex : public NodeBase {
public:
  AX_NODE_COMMON_WITH_CTOR(Normal_PerVertex, "Normal_PerVertex", "Computes per vertex normals",
                           register_render);
  AX_NODE_INPUTS((geo::SurfaceMesh, mesh, "The mesh to compute normals for"));
  AX_NODE_OUTPUTS((math::field3r, normals, "The normals of the mesh"));

  static void register_render() {
    add_custom_node_render<Normal_PerVertex>([](NodeBase* node) {
      begin_draw_node(node);
      draw_node_header_default(node);
      draw_node_content_default(node);
      auto* n = reinterpret_cast<Normal_PerVertex*>(node);
      ImGui::Text("Strategy: ");
      ImGui::SameLine();
      if (ImGui::Button(n->options_[n->selected_option_], ImVec2(100, 20))) {
        ImGui::OpenPopup("Normal_PerVertex_Options");
      }

      ed::Suspend();
      if (ImGui::BeginPopup("Normal_PerVertex_Options")) {
        for (int i = 0; i < 3; i++) {
          if (ImGui::Selectable(n->options_[i], n->selected_option_ == i)) {
            n->selected_option_ = i;
          }
        }
        ImGui::EndPopup();
      }
      ed::Resume();
      end_draw_node();
    });
  }

  void Apply(size_t ) override {
    auto const& mesh = AX_NODE_INPUT_ENSURE_EXTRACT(mesh);

    math::field3r normals;
    if (selected_option_ == 0) {
      normals = geo::normal_per_vertex(mesh.vertices_, mesh.indices_, geo::face_uniform_avg);
    } else if (selected_option_ == 1) {
      normals = geo::normal_per_vertex(mesh.vertices_, mesh.indices_, geo::face_area_avg);
    } else if (selected_option_ == 2) {
      normals = geo::normal_per_vertex(mesh.vertices_, mesh.indices_, geo::face_angle_avg);
    }
    SetOutput<math::field3r>(0, std::move(normals));
  }

  const char* options_[3]{"uniform", "area", "angle"};
  idx selected_option_ = 0;
};

class Make_XyPlane : public NodeBase {
public:
  AX_NODE_COMMON_WITH_CTOR(Make_XyPlane, "Make_XyPlane",
                           "Creates a xy plane (Default = [0,1]x[0,1] with 1x1 resolution)");
  AX_NODE_INPUTS((math::vec2r, size, "The size of the plane"),
                 (math::vec2i, resolution, "The resolution of the plane"));
  AX_NODE_OUTPUTS((geo::SurfaceMesh, mesh, "The resulting surface mesh"));

  void Apply(size_t ) override {
    math::vec2r size_inuse = AX_NODE_INPUT_EXTRACT_DEFAULT(size, math::vec2r(1, 1));
    math::vec2i resolution_inuse = AX_NODE_INPUT_EXTRACT_DEFAULT(resolution, math::vec2i(1, 1));
    if (resolution_inuse.x() < 1 || resolution_inuse.y() < 1) {
      throw make_out_of_range("Resolution must be at least 1x1.");
    }
    auto mesh
        = geo::plane(size_inuse.x(), size_inuse.y(), resolution_inuse.x(), resolution_inuse.y());
    SetOutput<geo::SurfaceMesh>(0, std::move(mesh));
  }
};

class ExtractBoundary_Tetrahedrons : public NodeBase {
public:
  AX_NODE_COMMON_WITH_CTOR(ExtractBoundary_Tetrahedrons, "ExtractBoundary_Tetrahedrons_pre",
                           "Extracts the boundary of a tetrahedral mesh");
  AX_NODE_INPUTS((math::field3r, vertices, "The vertices of the mesh"),
                 (math::field4i, tetras, "The tetrahedrons of the mesh"),
                 (bool, reload, "Whether to recompute the boundary every frame"));
  AX_NODE_OUTPUTS((math::field3i, bd_faces, "The boundary of the mesh"));

  void PreApply() override {
    const auto& V = AX_NODE_INPUT_ENSURE_EXTRACT(vertices);
    const auto& T = AX_NODE_INPUT_ENSURE_EXTRACT(tetras);

    if (V.cols() == 0 || T.cols() == 0) {
      throw make_invalid_argument("Vertices or tetrahedrons are empty.");
    }

    auto boundary = geo::get_boundary_triangles(V, T);
    SetOutput<math::field3i>(0, std::move(boundary));
  }

  void Apply(size_t frame) final {
    if (auto* reload = RetriveInput<bool>(2); (frame == 0) || (reload && *reload)) {
      PreApply();
    }
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
