#include "ax/geometry/common.hpp"
#include "ax/graph/render.hpp"
#include "ax/nodes/gl_prims.hpp"
#include "ax/graph/node.hpp"
#include "ax/gl/primitives/mesh.hpp"
#include "ax/utils/status.hpp"

#include <imnode/imgui_node_editor.h>

namespace ed = ax::NodeEditor;
using namespace ax;
using namespace graph;


namespace ax::nodes {

class Render_Mesh : public NodeBase {

public:
  Render_Mesh(NodeDescriptor const* descriptor, idx id) : NodeBase(descriptor, id) {}

  static void register_this() {
    NodeDescriptorFactory<Render_Mesh>()
        .SetName("Render_mesh")
        .SetDescription("Renders a mesh entity")
        .AddInput<entt::entity>("entity", "The entity to add the component")
        .AddInput<geo::SurfaceMesh>("mesh", "The mesh to render")
        .AddInput<math::field4r>("color", "The color of the mesh")
        .AddInput<math::vec4r>("u_color", "The color of the mesh")
        .AddInput<math::field3r>("normal", "The normal of the mesh")
        .AddInput<bool>("flat", "If true, the mesh will be rendered flat")
        .AddInput<bool>("use_lighting", "If true, the mesh will be rendered with lighting")
        .AddOutput<Entity>("entity", "The entity that has the mesh component")
        .AddOutput<gl::Mesh>("gl_mesh", "Mesh in render.")
        .FinalizeAndRegister();
  }

  Status Apply(idx) override {
    auto* entity = RetriveInput<entt::entity>(0);
    auto* mesh = RetriveInput<geo::SurfaceMesh>(1);
    auto* color = RetriveInput<math::field4r>(2);
    auto* u_color = RetriveInput<math::vec4r>(3);
    auto* normal = RetriveInput<math::field3r>(4);
    auto* flat = RetriveInput<bool>(5);
    auto* use_lighting = RetriveInput<bool>(6);

    if (mesh == nullptr) {
      return utils::FailedPreconditionError("mesh is not set.");
    }

    math::vec4r u_color_inuse = math::vec4r::Constant(0.7);
    if (u_color != nullptr) {
      u_color_inuse = *u_color;
    }

    Entity entity_inuse;

    if (entity != nullptr) {
      entity_inuse = *entity;
    } else {
      if (ent == entt::null) {
        ent = create_entity();
      }
      entity_inuse = ent;
    }

    auto& mesh_comp = add_or_replace_component<gl::Mesh>(entity_inuse);
    mesh_comp.vertices_ = mesh->vertices_;
    mesh_comp.indices_ = mesh->indices_;
    if (normal != nullptr) {
      mesh_comp.normals_ = *normal;
    }

    if (color == nullptr) {
      mesh_comp.colors_ = math::field4r(4, mesh->vertices_.size());
      mesh_comp.colors_.colwise() = u_color_inuse;
    } else {
      mesh_comp.colors_ = *color;
      if (u_color != nullptr) {
        mesh_comp.colors_.colwise() = *u_color;
      }
    }

    if (flat != nullptr) {
      mesh_comp.is_flat_ = *flat;
    }

    if (use_lighting != nullptr) {
      mesh_comp.use_lighting_ = *use_lighting;
    }

    SetOutput<gl::Mesh>(1, mesh_comp);
    SetOutput<Entity>(0, entity_inuse);

    AX_RETURN_OK();
  }

  Status OnDestroy() override {
    if (ent != entt::null) {
      destroy_entity(ent);
    }
    AX_RETURN_OK();
  }
  Entity ent = entt::null;
};


void register_gl_prim_nodes(){
  Render_Mesh::register_this();
}

}
