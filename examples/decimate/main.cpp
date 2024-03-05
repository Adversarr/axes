#include <absl/flags/declare.h>
#include <absl/flags/flag.h>
#include <igl/readOBJ.h>
#include <imgui.h>

#include <axes/gl/utils.hpp>
#include <fstream>

#include "axes/core/echo.hpp"
#include "axes/core/entt.hpp"
#include "axes/core/init.hpp"
#include "axes/geometry/decimate.hpp"
#include "axes/geometry/halfedge.hpp"
#include "axes/geometry/io.hpp"
#include "axes/geometry/normal.hpp"
#include "axes/gl/context.hpp"
#include "axes/gl/primitives/lines.hpp"
#include "axes/gl/primitives/mesh.hpp"
#include "axes/gl/window.hpp"
#include "axes/utils/asset.hpp"

using namespace ax;

ABSL_FLAG(std::string, obj_file, "jelly_low_res.obj", "The obj file to load");

float ratio = 1.0f;

const char* opt[2] = {"SolveQuadric", "DirectHalf"};

int option = 0;

std::string file;

entt::entity original, modified;
math::field3r vertices;
math::field3i indices;

void ui_render_callback(gl::UiRenderEvent) {
  ImGui::SetNextWindowPos(ImVec2(10, 10), ImGuiCond_FirstUseEver);
  ImGui::Begin("Decimate", nullptr, ImGuiWindowFlags_AlwaysAutoResize);
  auto& modified_mesh = ax::get_component<gl::Mesh>(modified);

  ImGui::Text("Input File: %s", file.c_str());

  ImGui::Text("Original Mesh");
  ImGui::BulletText("Number of faces: %ld, Number of vertices: %ld", indices.cols(),
                    vertices.cols());

  ImGui::Text("Modified Mesh");
  ImGui::BulletText("Number of faces: %ld, Number of vertices: %ld", modified_mesh.indices_.cols(),
                    modified_mesh.vertices_.cols());

  ImGui::Combo("Collapse Strategy", &option, opt, IM_ARRAYSIZE(opt));

  ImGui::SliderFloat("Ratio", &ratio, 0.0f, 1.0f);
  if (ImGui::Button("Run Algorithm")) {
    auto halfedge = geo::HalfedgeMesh(vertices, indices);
    auto beg_time = absl::Now();
    auto decimator = geo::MeshDecimator(&halfedge);
    decimator.SetRatio(ratio);
    if (option == 0) {
      decimator.SetStrategy(geo::MeshDecimator::kQuadratic);
    } else {
      decimator.SetStrategy(geo::MeshDecimator::kDirect);
    }
    AX_CHECK_OK(decimator.Run());
    auto end_time = absl::Now();

    AX_LOG(INFO) << "Time elapsed: " << absl::ToDoubleSeconds(end_time - beg_time) << "s";

    std::tie(modified_mesh.vertices_, modified_mesh.indices_) = halfedge.ToTriangleMesh();
    modified_mesh.colors_.resize(4, modified_mesh.vertices_.cols());
    modified_mesh.colors_.setConstant(1);
    modified_mesh.colors_.topRows(3) = modified_mesh.vertices_;
    modified_mesh.normals_
        = geo::normal_per_vertex(modified_mesh.vertices_, modified_mesh.indices_);
    modified_mesh.flush_ = modified_mesh.use_global_model_ = true;

    auto& wf = ax::get_component<gl::Lines>(modified);  // Wireframe
    wf = gl::Lines::Create(modified_mesh);
    wf.flush_ = true;
    wf.colors_.setOnes();
    wf.instance_color_.setOnes();
  }

  ImGui::End();
}

int main(int argc, char** argv) {
  ax::gl::init(argc, argv);
  auto& ctx = get_resource<gl::Context>();

  connect<gl::UiRenderEvent, &ui_render_callback>();

  auto mesh_ent = create_entity();
  modified = mesh_ent;

  file = utils::get_asset("/mesh/obj/" + absl::GetFlag(FLAGS_obj_file));
  AX_LOG(INFO) << "Reading " << std::quoted(file);
  auto obj_result = geo::read_obj(file);
  if (!obj_result.ok()) {
    AX_LOG(ERROR) << "Failed to read obj file in asset folder: " << obj_result.status();
    file = absl::GetFlag(FLAGS_obj_file);
    AX_LOG(INFO) << "Reading " << std::quoted(file);
    obj_result = geo::read_obj(file);
  }
  AX_CHECK_OK(obj_result) << "Failed to read obj file: " << file;
  std::tie(vertices, indices) = obj_result.value();

  AX_LOG(INFO) << "Mesh Stat: #V=" << vertices.cols() << ", #F=" << indices.cols();

  auto row_y = vertices.row(1).eval();
  auto row_z = vertices.row(2).eval();
  vertices.row(1) = row_z;
  vertices.row(2) = row_y;

  math::vec3r min_xyz = vertices.rowwise().minCoeff();
  math::vec3r max_xyz = vertices.rowwise().maxCoeff();
  {  // Setup Original
    original = create_entity();
    auto& mesh = add_component<gl::Mesh>(original);
    std::tie(mesh.vertices_, mesh.indices_) = std::make_pair(vertices, indices);
    mesh.normals_ = geo::normal_per_vertex(mesh.vertices_, mesh.indices_);
    mesh.colors_.resize(4, mesh.vertices_.cols());
    mesh.colors_.setConstant(1);
    mesh.colors_.topRows(3) = mesh.vertices_;
    mesh.flush_ = mesh.use_global_model_ = true;
    mesh.vertices_.row(0).array() += 1.1 * (max_xyz - min_xyz).x();
    mesh.use_lighting_ = false;

    auto& wf = add_component<gl::Lines>(original, gl::Lines::Create(mesh));  // Wireframe
    wf.colors_.setOnes();
    wf.instance_color_.setOnes();
  }
  geo::HalfedgeMesh he_mesh(vertices, indices);
  std::vector<geo::HalfedgeEdge_t*> edge_to_collapse;
  AX_CHECK(he_mesh.CheckOk()) << "The mesh is not a manifold.";

  auto& mesh = add_component<gl::Mesh>(mesh_ent);

  std::tie(mesh.vertices_, mesh.indices_) = he_mesh.ToTriangleMesh();
  mesh.normals_ = geo::normal_per_vertex(mesh.vertices_, mesh.indices_);
  mesh.colors_.resize(4, mesh.vertices_.cols());
  mesh.colors_.setConstant(1);
  mesh.colors_.topRows(3) = mesh.vertices_;
  mesh.flush_ = mesh.use_global_model_ = true;

  mesh.use_lighting_ = false;
  auto& mesh_wireframe = add_component<gl::Lines>(mesh_ent, gl::Lines::Create(mesh));  // Wireframe
  mesh_wireframe.colors_.setOnes();
  mesh_wireframe.instance_color_.setOnes();

  auto const& win = ctx.GetWindow();

  ctx.GetCamera().SetProjectionMode(true);
  while (!win.ShouldClose()) {
    AX_CHECK_OK(ctx.TickLogic());
    AX_CHECK_OK(ctx.TickRender());
  }

  ax::clean_up();
  return 0;
}
