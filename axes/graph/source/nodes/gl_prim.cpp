#include "ax/geometry/common.hpp"
#include "ax/gl/colormap.hpp"
#include "ax/graph/render.hpp"
#include "ax/nodes/gl_prims.hpp"
#include "ax/graph/node.hpp"
#include "ax/gl/primitives/mesh.hpp"
#include "ax/utils/status.hpp"

#include <imgui_node_editor.h>

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

class ColorMap_real : public NodeBase {
public:
  ColorMap_real(NodeDescriptor const* descriptor, idx id) : NodeBase(descriptor, id) {}

  static void register_this() {
    NodeDescriptorFactory<ColorMap_real>()
        .SetName("Colormap_real")
        .SetDescription("Maps a real value to a color")
        .AddInput<real>("value", "The real value to map")
        .AddInput<real>("low", "Low value of the map")
        .AddInput<real>("high", "High value of the map")
        .AddOutput<math::vec4r>("color", "The color mapped from the value")
        .FinalizeAndRegister();
  }

  Status Apply(idx) override {
    auto* value = RetriveInput<real>(0);
    auto* low = RetriveInput<real>(1);
    auto* high = RetriveInput<real>(2);

    if (value == nullptr) {
      return utils::FailedPreconditionError("value is not set.");
    }

    if (low == nullptr) {
      return utils::FailedPreconditionError("low is not set.");
    }

    if (high == nullptr) {
      return utils::FailedPreconditionError("high is not set.");
    }

    real value_inuse = *value;
    real low_inuse = *low;
    real high_inuse = *high;

    gl::Colormap cmap(low_inuse, high_inuse);
    auto rgb = cmap(value_inuse);
    math::vec4r rgba;
    rgba << rgb, 1;
    SetOutput<math::vec4r>(0, rgba);
    AX_RETURN_OK();
  }
};

class ColorMap_Normal : public NodeBase {
public:
  ColorMap_Normal(NodeDescriptor const* descriptor, idx id) : NodeBase(descriptor, id) {}

  static void register_this() {
    NodeDescriptorFactory<ColorMap_Normal>()
        .SetName("Colormap_normal")
        .SetDescription("Maps a normal to a color")
        .AddInput<math::field3r>("normal", "The normal to map")
        .AddOutput<math::field4r>("color", "The color mapped from the normal")
        .FinalizeAndRegister();
  }

  Status Apply(idx) override {
    auto* normal = RetriveInput<math::field3r>(0);

    if (normal == nullptr) {
      return utils::FailedPreconditionError("normal is not set.");
    }

    math::field3r const& normal_inuse = *normal;
    math::field4r out(4, normal_inuse.cols());

    out.topRows(3) = normal_inuse;
    out.row(3).setOnes();
    SetOutput<math::field4r>(0, std::move(out));
    AX_RETURN_OK();
  }
};

void register_gl_prim_nodes(){
  Render_Mesh::register_this();
  ColorMap_real::register_this();
  ColorMap_Normal::register_this();
}

}
