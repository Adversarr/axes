#include <imgui_node_editor.h>
#include <iostream>

#include "ax/core/entt.hpp"
#include "ax/core/excepts.hpp"
#include "ax/geometry/common.hpp"
#include "ax/gl/colormap.hpp"
#include "ax/gl/primitives/lines.hpp"
#include "ax/gl/primitives/mesh.hpp"
#include "ax/gl/primitives/points.hpp"
#include "ax/graph/cache_sequence.hpp"
#include "ax/graph/node.hpp"
#include "ax/graph/render.hpp"
#include "ax/nodes/gl_prims.hpp"

namespace ed = ax::NodeEditor;
using namespace ax;
using namespace graph;

namespace ax::nodes {

const char* cmap_names[] = {
    "bwr", "coolwarm", "jet", "plasma", "seismic",
};
gl::cmap& get_colormap(idx which) {
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
        .AddInput<bool>("flush", "If true, the mesh will be rendered to screen.")
        .AddOutput<Entity>("entity", "The entity that has the mesh component")
        .AddOutput<gl::Mesh>("gl_mesh", "Mesh in render.")
        .FinalizeAndRegister();
  }

  void Apply(size_t) override {
    auto* entity = RetriveInput<entt::entity>(0);
    auto* mesh = RetriveInput<geo::SurfaceMesh>(1);
    auto* color = RetriveInput<math::field4r>(2);
    auto* u_color = RetriveInput<math::vec4r>(3);
    auto* normal = RetriveInput<math::field3r>(4);
    auto* flat = RetriveInput<bool>(5);
    auto* use_lighting = RetriveInput<bool>(6);
    auto* flush = RetriveInput<bool>(7);

    if (mesh == nullptr) {
      throw make_invalid_argument("mesh is not set.");
    }

    math::vec4r u_color_inuse = math::vec4r::Constant(0.);
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

    mesh_comp.colors_ = math::field4r::Constant(4, mesh->vertices_.size(), 0.7);
    if (color != nullptr) {
      mesh_comp.colors_ = *color;
    }
    mesh_comp.colors_.colwise() += u_color_inuse;

    if (flat != nullptr) {
      mesh_comp.is_flat_ = *flat;
    }

    if (use_lighting != nullptr) {
      mesh_comp.use_lighting_ = *use_lighting;
    }

    if (flush != nullptr) {
      mesh_comp.flush_ = *flush;
    }

    SetOutput<gl::Mesh>(1, mesh_comp);
    SetOutput<Entity>(0, entity_inuse);
  }

  void CleanUp() override {
    if (ent != entt::null) {
      destroy_entity(ent);
    } else {
      if (entity_inuse != entt::null) {
        if (global_registry().valid(entity_inuse) && has_component<gl::Mesh>(entity_inuse)) {
          remove_component<gl::Mesh>(entity_inuse);
        }
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

  void Apply(size_t) override {
    auto* entity = RetriveInput<entt::entity>(0);
    auto* vertices = RetriveInput<math::field3r>(1);
    auto* indices = RetriveInput<math::field2i>(2);
    auto* color = RetriveInput<math::field4r>(3);
    auto* u_color = RetriveInput<math::vec4r>(4);

    math::field2i indices_in_use;

    if (vertices == nullptr) {
      if (entity == nullptr) {
        // return utils::FailedPreconditionError("points is not set.");
        throw make_invalid_argument("points is not set.");
      }
      auto* msh = try_get_component<gl::Mesh>(*entity);
      if (msh == nullptr) {
        throw make_invalid_argument("The input entity does not have a mesh");
      } else {
        auto& l = add_or_replace_component<gl::Lines>(*entity, gl::Lines::Create(*msh));
        entity_inuse = *entity;
        SetOutput<Entity>(0, *entity);
        SetOutput<gl::Lines>(1, l);
        return;
      }
    }

    if (indices == nullptr) {
      indices_in_use = math::field2i(2, vertices->cols() - 1);
      for (idx i = 0; i < vertices->cols() - 1; ++i) {
        indices_in_use(0, i) = i;
        indices_in_use(1, i) = i + 1;
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
    lines_comp.vertices_ = *vertices;
    lines_comp.indices_ = std::move(indices_in_use);
    
    lines_comp.colors_ = math::field4r::Zero(4, vertices->cols());
    if (color != nullptr) {
      lines_comp.colors_ = *color;
    }
    if (u_color != nullptr) {
      lines_comp.colors_.colwise() += *u_color;
    }

    SetOutput<Entity>(0, entity_inuse);
    SetOutput<gl::Lines>(1, lines_comp);
  }

  void CleanUp() override {
    if (ent != entt::null) {
      destroy_entity(ent);
    } else {
      if (entity_inuse != entt::null) {
        if (global_registry().valid(entity_inuse) && has_component<gl::Lines>(entity_inuse)) {
          remove_component<gl::Lines>(entity_inuse);
        }
      }
    }
  }

  Entity ent = entt::null;
  Entity entity_inuse = entt::null;
};

class Render_Points : public NodeBase {
public:
  Render_Points(NodeDescriptor const* descriptor, idx id) : NodeBase(descriptor, id) {}

  static void register_this() {
    NodeDescriptorFactory<Render_Points>()
        .SetName("Render_Points")
        .SetDescription("Renders a point entity")
        .AddInput<Entity>("entity", "The entity to add the component")
        .AddInput<math::field3r>("vertices", "The points to render")
        .AddInput<math::field4r>("color", "The color of the points")
        .AddInput<math::vec4r>("u_color", "The color of the points")
        .AddInput<bool>("flush", "Flush the points to screen.")
        .AddOutput<Entity>("entity", "The entity that has the point component")
        .AddOutput<gl::Points>("gl_points", "Points in render")
        .FinalizeAndRegister();
  }

  void Apply(size_t) override {
    auto* entity = RetriveInput<entt::entity>(0);
    auto* points = RetriveInput<math::field3r>(1);
    auto* color = RetriveInput<math::field4r>(2);
    auto* u_color = RetriveInput<math::vec4r>(3);

    if (points == nullptr) {
      throw make_invalid_argument("points is not set.");
    }

    if (entity != nullptr) {
      entity_inuse = *entity;
    } else {
      if (ent == entt::null) {
        ent = create_entity();
      }
      entity_inuse = ent;
    }

    auto& points_comp = add_or_replace_component<gl::Points>(entity_inuse);
    points_comp.vertices_ = *points;
    if (color == nullptr) {
      points_comp.colors_ = math::field4r::Constant(4, points->cols(), 0.7);
    } else {
      if (color->cols() != points->cols()) {
        throw make_invalid_argument("The number of colors does not match the number of points.");
      }
      points_comp.colors_ = *color;
    }
    if (u_color != nullptr) {
      points_comp.colors_.colwise() += *u_color;
    }

    SetOutput<gl::Points>(1, points_comp);
    SetOutput<Entity>(0, entity_inuse);
  }

  void CleanUp() override {
    if (ent != entt::null) {
      destroy_entity(ent);
    } else {
      if (entity_inuse != entt::null) {
        if (global_registry().valid(entity_inuse) && has_component<gl::Points>(entity_inuse)) {
          remove_component<gl::Points>(entity_inuse);
        }
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
    add_custom_node_render<ColorMap_real>([](NodeBase* node) {
      draw_node_content_default(node);
      draw_node_header_default(node);
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

  void Apply(size_t) override {
    auto* value = RetriveInput<real>(0);
    auto* low = RetriveInput<real>(1);
    auto* high = RetriveInput<real>(2);

    if (value == nullptr) {
      throw make_invalid_argument("value is not set.");
    }

    if (low == nullptr) {
      throw make_invalid_argument("low is not set.");
    }

    if (high == nullptr) {
      throw make_invalid_argument("high is not set.");
    }

    real value_inuse = *value;
    real low_inuse = *low;
    real high_inuse = *high;

    gl::Colormap cmap(low_inuse, high_inuse);
    auto rgb = cmap(value_inuse);
    math::vec4r rgba;
    rgba << rgb, 1;
    SetOutput<math::vec4r>(0, rgba);
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
      if (which_map < 0 || static_cast<size_t>(which_map) > sizeof(cmap_names) / sizeof(cmap_names[0])) {
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
        .AddInput<math::field1r>("data", "The field to map")
        .AddOutput<math::field4r>("color", "The color mapped from the field")
        .FinalizeAndRegister();
  }

  void Apply(size_t) override {
    auto* field = RetriveInput<math::field1r>(0);

    if (field == nullptr) {
      throw make_invalid_argument("field is not set.");
    }

    math::field1r const& field_inuse = *field;
    math::field4r out(4, field_inuse.cols());

    out.topRows(3) = field_inuse.replicate(3, 1);
    out.row(3).setOnes();
    SetOutput<math::field4r>(0, std::move(out));
  }
};

class GlMeshCachedSequence : public NodeBase {
public:
  GlMeshCachedSequence(NodeDescriptor const* descriptor, idx id) : NodeBase(descriptor, id) {}

  static void register_this() {
    NodeDescriptorFactory<GlMeshCachedSequence>()
        .SetName("GlMeshCachedSequence")
        .SetDescription("Renders a sequence of mesh entities")
        .AddInput<gl::Mesh>("mesh", "The mesh to render")
        .AddInput<bool>("flush", "Display the input mesh directly.")
        .FinalizeAndRegister();
  }

  void OnUpdate(CacheSequenceUpdateEvent const& event) {
    std::cout << "Updating..." << std::endl;
    if (event.is_cleanup_) {
      meshes.clear();
      return;
    }
    size_t to_show = event.required_frame_id_;
    if (to_show >= meshes.size()) {
      return;
    }
    auto& mesh = meshes[to_show];
    mesh.flush_ = true;
    add_or_replace_component<gl::Mesh>(ent_created, mesh);
  }

  void OnConstruct() override {
    connect<CacheSequenceUpdateEvent, &GlMeshCachedSequence::OnUpdate>(this);
    ent_created = create_entity();
  }

  void OnDestroy() override {
    destroy_entity(ent_created);
    disconnect<CacheSequenceUpdateEvent, &GlMeshCachedSequence::OnUpdate>(this);
  }

  void Apply(size_t) override {
    auto* mesh = RetriveInput<gl::Mesh>(0);
    if (mesh == nullptr) {
      throw make_invalid_argument("Mesh is not set.");
    }
    meshes.push_back(*mesh);
    if (auto flush = RetriveInput<bool>(1); flush != nullptr) {
      meshes.back().flush_ = *flush;
    }
  }

  std::vector<gl::Mesh> meshes;

  Entity ent_created;
};

void register_gl_prim_nodes() {
  Render_Mesh::register_this();
  Render_Lines::register_this();
  Render_Points::register_this();

  ColorMap_real::register_this();
  ColorMap_field1r::register_this();
  GlMeshCachedSequence::register_this();
}

}  // namespace ax::nodes
