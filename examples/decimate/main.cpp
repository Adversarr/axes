#include <absl/flags/declare.h>
#include <absl/flags/flag.h>
#include <imgui.h>

#include <fstream>

#include "axes/core/echo.hpp"
#include "axes/core/entt.hpp"
#include "axes/core/init.hpp"
#include "axes/geometry/decimate.hpp"
#include "axes/geometry/halfedge.hpp"
#include "axes/geometry/normal.hpp"
#include "axes/gl/context.hpp"
#include "axes/gl/primitives/lines.hpp"
#include "axes/gl/primitives/mesh.hpp"
#include "axes/gl/window.hpp"
#include "axes/utils/asset.hpp"

#include <igl/readOBJ.h>

using namespace ax;

ABSL_FLAG(std::string, obj_file, "jelly_low_res_remesh.obj", "The obj file to load");

float ratio = 1.0f;

entt::entity original, modified;
math::field3r vertices;
math::field3i indices;

void ui_render_callback(gl::UiRenderEvent) {
  ImGui::SetNextWindowPos(ImVec2(10, 10), ImGuiCond_FirstUseEver);
  ImGui::Begin("Decimate");
  ImGui::SliderFloat("Ratio", &ratio, 0.0f, 1.0f);
  bool run_algorithm = ImGui::Button("Run Algorithm");
  if (run_algorithm) {
    auto& modified_mesh = ax::get_component<gl::Mesh>(modified);
    auto halfedge = geo::HalfedgeMesh(vertices, indices);

    auto decimator = geo::MeshDecimator(&halfedge);
    decimator.SetRatio(ratio);
    AX_CHECK_OK(decimator.Run());
    std::tie(modified_mesh.vertices_, modified_mesh.indices_) = halfedge.ToTriangleMesh();
    modified_mesh.colors_.resize(4, modified_mesh.vertices_.cols());
    modified_mesh.colors_.setConstant(0.5);
    modified_mesh.colors_.topRows(3) = modified_mesh.vertices_;
    modified_mesh.normals_ = geo::normal_per_vertex(modified_mesh.vertices_, modified_mesh.indices_);
    modified_mesh.flush_ =  modified_mesh.use_global_model_ = true;

    auto& wf = ax::get_component<gl::Lines>(modified);  // Wireframe
    wf = gl::Lines::Create(modified_mesh);
    wf.flush_ = true;
    wf.colors_.setOnes();
    wf.instance_color_.setOnes();
  }
  ImGui::End();
}


std::pair<math::field3i, math::field3r> load_obj(const std::string& filename) {
  std::vector<math::vec3r> vertices_list;
  std::vector<math::vec3i> indices_list;
  math::field3r vertices;
  math::field3i indices;

  math::matr<Eigen::Dynamic, 3> V;
  math::mati<Eigen::Dynamic, 3> F;
  igl::readOBJ(utils::get_asset("/mesh/obj/" + filename), V, F);
  vertices = V.transpose();
  indices = F.transpose().cast<idx>();

  return std::make_pair(indices, vertices);
}

int main(int argc, char** argv) {
  ax::init(argc, argv);
  auto& ctx = add_resource<gl::Context>();

  connect<gl::UiRenderEvent, &ui_render_callback>();

  auto mesh_ent = create_entity();
  modified = mesh_ent;

  auto file = absl::GetFlag(FLAGS_obj_file);
  AX_LOG(INFO) << "Reading " << std::quoted(file);
  std::tie(indices, vertices) = load_obj(file);

  auto row_y = vertices.row(1).eval();
  auto row_z = vertices.row(2).eval();
  vertices.row(1) = row_z;
  vertices.row(2) = row_y;

  math::vec3r min_xyz = vertices.rowwise().minCoeff();
  math::vec3r max_xyz = vertices.rowwise().maxCoeff();
  { // Setup Original
    original = create_entity();
    auto& mesh = add_component<gl::Mesh>(original);
    std::tie(mesh.vertices_, mesh.indices_) = std::make_pair(vertices, indices);
    mesh.normals_ = geo::normal_per_vertex(mesh.vertices_, mesh.indices_);
    mesh.colors_.resize(4, mesh.vertices_.cols());
    mesh.colors_.setConstant(0.5);
    mesh.colors_.topRows(3) = mesh.vertices_;
    mesh.flush_ =  mesh.use_global_model_ = true;
    mesh.vertices_.row(0).array() += (max_xyz - min_xyz).x();
    mesh.use_lighting_ = false;

    auto& wf = add_component<gl::Lines>(original, gl::Lines::Create(mesh));  // Wireframe
    wf.colors_.setOnes();
    wf.instance_color_.setOnes();
  }
  geo::HalfedgeMesh he_mesh(vertices, indices);
  std::vector<geo::HalfedgeEdge_t* > edge_to_collapse;
  he_mesh.CheckOk();

  auto& mesh = add_component<gl::Mesh>(mesh_ent);

  std::tie(mesh.vertices_, mesh.indices_) = he_mesh.ToTriangleMesh();
  mesh.normals_ = geo::normal_per_vertex(mesh.vertices_, mesh.indices_);
  mesh.colors_.resize(4, mesh.vertices_.cols());
  mesh.colors_.setConstant(0.5);
  mesh.colors_.topRows(3) = mesh.vertices_;
  mesh.flush_ =  mesh.use_global_model_ = true;

  mesh.use_lighting_ = false;
  auto& mesh_wireframe = add_component<gl::Lines>(mesh_ent, gl::Lines::Create(mesh));  // Wireframe
  mesh_wireframe.colors_.setOnes();
  mesh_wireframe.instance_color_.setOnes();


  auto& win = ctx.GetWindow();

  ctx.GetCamera().SetProjectionMode(true);
  while (!win.ShouldClose()) {
    AX_CHECK_OK(ctx.TickLogic());
    AX_CHECK_OK(ctx.TickRender());
  }

  erase_resource<gl::Context>();
  ax::clean_up();
  return 0;
}
