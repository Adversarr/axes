#include <absl/flags/declare.h>
#include <absl/flags/flag.h>
#include <imgui.h>

#include <fstream>

#include "axes/core/echo.hpp"
#include "axes/core/entt.hpp"
#include "axes/core/init.hpp"
#include "axes/geometry/halfedge.hpp"
#include "axes/geometry/normal.hpp"
#include "axes/geometry/primitives.hpp"
#include "axes/geometry/transforms.hpp"
#include "axes/gl/colormap.hpp"
#include "axes/gl/context.hpp"
#include "axes/gl/extprim/axes.hpp"
#include "axes/gl/primitives/lines.hpp"
#include "axes/gl/primitives/mesh.hpp"
#include "axes/gl/window.hpp"
#include "axes/utils/asset.hpp"
#include "axes/utils/time.hpp"

using namespace ax;

std::pair<math::field3i, math::field3r> load_obj(const std::string& filename) {
  std::vector<math::vec3r> vertices_list;
  std::vector<math::vec3i> indices_list;
  math::field3r vertices;
  math::field3i indices;
  std::ifstream file(utils::get_asset("/mesh/obj/" + filename));
  AX_CHECK(file.is_open());

  for (std::string line; std::getline(file, line);) {
    std::istringstream iss(line);
    std::string type;
    iss >> type;
    if (type == "v") {
      math::vec3r v;
      iss >> v.x() >> v.y() >> v.z();
      vertices_list.push_back(v);
    } else if (type == "f") {
      math::vec3i f;
      iss >> f.x() >> f.y() >> f.z();
      f -= math::ones<3, 1, idx>();
      indices_list.push_back(f);
    }
  }

  vertices.resize(3, vertices_list.size());
  for (idx i = 0; i < vertices_list.size(); ++i) {
    vertices.col(i) = vertices_list[i];
  }

  indices.resize(3, indices_list.size());
  for (idx i = 0; i < indices_list.size(); ++i) {
    indices.col(i) = indices_list[i];
  }

  return std::make_pair(indices, vertices);
}

int main(int argc, char** argv) {
  ax::init(argc, argv);
  auto& ctx = add_resource<gl::Context>();

  // SECT: This is a dummy entity to test the rendering pipeline
  auto mesh_ent = create_entity();

  auto [indices, vertices] = load_obj("box_naive.obj");
  gl::Mesh mesh;
  geo::HalfedgeMesh halfedge_mesh(vertices, indices);
  halfedge_mesh.ForeachEdge([&](geo::HalfedgeEdge_t* edge) {
    AX_LOG(INFO) << "Edge is collapsable? " << (halfedge_mesh.CheckCollapse(edge));
  });
  halfedge_mesh.CollapseEdge(halfedge_mesh.edges_[1].get(), math::vec3r{0, -1, 0});
  std::tie(mesh.vertices_, mesh.indices_) = halfedge_mesh.ToTriangleMesh();

  mesh.normals_ = geo::normal_per_vertex(mesh.vertices_, mesh.indices_);
  mesh.colors_.resize(4, mesh.vertices_.cols());
  mesh.colors_.setConstant(0.5);
  mesh.colors_.topRows(3) = mesh.vertices_;
  mesh.flush_ =  mesh.use_global_model_ = true;
  // mesh.is_flat_ = true;
  mesh.use_lighting_ = false;
  auto& m = add_component<gl::Mesh>(mesh_ent, mesh);
  auto& mesh_wireframe = add_component<gl::Lines>(mesh_ent, gl::Lines::Create(m));  // Wireframe
  mesh_wireframe.colors_.setOnes();
  mesh_wireframe.instance_color_.setOnes();
  // SECT: Main Loop
  auto& win = ctx.GetWindow();
  i64 start = utils::GetCurrentTimeNanos();



  ctx.GetCamera().SetProjectionMode(true);
  while (!win.ShouldClose()) {
    AX_CHECK_OK(ctx.TickLogic());
    AX_CHECK_OK(ctx.TickRender());
  }

  // NOTE: Clean Up
  erase_resource<gl::Context>();
  clean_up();
  return 0;
}
