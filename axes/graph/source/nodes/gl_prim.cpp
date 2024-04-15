#include "ax/geometry/common.hpp"
#include "ax/gl/colormap.hpp"
#include "ax/gl/primitives/lines.hpp"
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

const char * cmap_names[] = {
  "bwr",
  "coolwarm",
  "jet",
  "plasma",
  "seismic",
};
gl::cmap & get_colormap(idx which) {
  switch (which) {
    case 0:
      return gl::colormap_bwr;
    case 1:
      return gl::colormap_coolwarm;
    case 2:
      return gl::colormap_jet;
    case 3:
      return gl::colormap_plasma;
    case 4:
      return gl::colormap_seismic;
    default:
      return gl::colormap_jet;
  }
}

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

  void CleanUp() override {
    if (ent != entt::null) {
      destroy_entity(ent);
    } else {
      if (entity_inuse != entt::null) {
        remove_component<gl::Mesh>(entity_inuse);
      }
    }
  }
  Entity ent = entt::null;
  Entity entity_inuse = entt::null;
};

class Render_Lines : public NodeBase {
public:
  Render_Lines(NodeDescriptor const* descriptor, idx id) : NodeBase(descriptor, id) {}

  static void register_this() {
    NodeDescriptorFactory<Render_Lines>()
        .SetName("Render_Lines")
        .SetDescription("Renders a line entity")
        .AddInput<entt::entity>("entity", "The entity to add the component")
        .AddInput<math::field3r>("vertices", "The points to render")
        .AddInput<math::field2i>("indices", "The indices of the points")
        .AddInput<math::field4r>("color", "The color of the line")
        .AddInput<math::vec4r>("u_color", "The color of the line")
        .AddOutput<Entity>("entity", "The entity that has the line component")
        .AddOutput<gl::Lines>("gl_lines", "Lines in render")
        .FinalizeAndRegister();
  }

  Status Apply(idx) override {
    auto* entity = RetriveInput<entt::entity>(0);
    auto* points = RetriveInput<math::field3r>(1);
    auto* indices = RetriveInput<math::field2i>(2);
    auto* color = RetriveInput<math::field4r>(3);
    auto* u_color = RetriveInput<math::vec4r>(4);

    math::field2i indices_in_use;

    if (points == nullptr) {
      if (entity == nullptr) {
        return utils::FailedPreconditionError("points is not set.");
      }
      auto* msh = try_get_component<gl::Mesh>(*entity);
      if (msh == nullptr) {
        return utils::FailedPreconditionError("The input entity does not have a mesh");
      } else {
        auto& l = add_or_replace_component<gl::Lines>(*entity, gl::Lines::Create(*msh));
        entity_inuse = *entity;
        SetOutput<Entity>(0, *entity);
        SetOutput<gl::Lines>(1, l);
        AX_RETURN_OK();
      }
    }

    if (indices == nullptr) {
      indices_in_use = math::field2i(2, points->cols() / 2);
      for (idx i = 0; i < indices_in_use.cols(); i++) {
        indices_in_use(0, i) = 2 * i;
        indices_in_use(1, i) = 2 * i + 1;
      }
    } else {
      indices_in_use = *indices;
    }
    if (entity != nullptr) {
      entity_inuse = *entity;
    } else {
      if (ent == entt::null) {
        ent = create_entity();
      }
      entity_inuse = ent;
    }

    auto& lines_comp = add_or_replace_component<gl::Lines>(entity_inuse);
    lines_comp.vertices_ = *points;
    lines_comp.indices_ = std::move(indices_in_use);
    if (color != nullptr) {
      lines_comp.colors_ = *color;
    } else if (u_color != nullptr) {
      lines_comp.colors_ = math::field4r(4, points->cols());
      lines_comp.colors_.colwise() = *u_color;
    }

    SetOutput<Entity>(0, entity_inuse);
    SetOutput<gl::Lines>(1, lines_comp);
    AX_RETURN_OK();
  }

  void CleanUp() override {
    if (ent != entt::null) {
      destroy_entity(ent);
    } else {
      if (entity_inuse != entt::null) {
        remove_component<gl::Lines>(entity_inuse);
      }
    }
  }

  Entity ent = entt::null;
  Entity entity_inuse = entt::null;
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
    add_custom_node_render<ColorMap_real>([](NodeBase *node) {
      draw_node_content_default(node);
      ImGui::Text("Colormap:");
      ImGui::SameLine();
      idx& n = static_cast<ColorMap_real*>(node)->which_map;
      if (ImGui::Button(cmap_names[n])) {
        ImGui::OpenPopup("colormap_select");
      }

      ed::Suspend();
      if (ImGui::BeginPopup("colormap_select")) {
        for (idx i = 0; i < 5; i++) {
          if (ImGui::Selectable(cmap_names[i])) {
            n = i;
          }
        }
        ImGui::EndPopup();
      }
      ed::Resume();
    });
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

  boost::json::object Serialize() const override {
    auto obj = NodeBase::Serialize();
    obj["which_map"] = which_map;
    return obj;
  }
  idx which_map = 0;

  void Deserialize(boost::json::object const& obj) override {
    if (obj.contains("which_map")) {
      which_map = obj.at("which_map").as_int64();
      if (which_map < 0 || (size_t) which_map > sizeof(cmap_names) / sizeof(cmap_names[0])) {
        which_map = 0;
      }
    }
  }
};


class ColorMap_field1r : public NodeBase {
public:
  ColorMap_field1r(NodeDescriptor const* descriptor, idx id) : NodeBase(descriptor, id) {}

  static void register_this() {
    NodeDescriptorFactory<ColorMap_field1r>()
        .SetName("Colormap_field1r")
        .SetDescription("Maps a field1r to a color")
        .AddInput<math::field1r>("field", "The field to map")
        .AddOutput<math::field4r>("color", "The color mapped from the field")
        .FinalizeAndRegister();
  }

  Status Apply(idx) override {
    auto* field = RetriveInput<math::field1r>(0);

    if (field == nullptr) {
      return utils::FailedPreconditionError("field is not set.");
    }

    math::field1r const& field_inuse = *field;
    math::field4r out(4, field_inuse.cols());

    out.topRows(3) = field_inuse.replicate(3, 1);
    out.row(3).setOnes();
    SetOutput<math::field4r>(0, std::move(out));
    AX_RETURN_OK();
  }
};

void register_gl_prim_nodes(){
  Render_Mesh::register_this();
  Render_Lines::register_this();
  ColorMap_real::register_this();
  ColorMap_field1r::register_this();
}

}
