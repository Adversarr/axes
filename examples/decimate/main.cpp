#include <absl/flags/declare.h>
#include <absl/flags/flag.h>
#include <igl/readOBJ.h>
#include <imgui.h>

#include <ax/gl/utils.hpp>

#include "ax/core/echo.hpp"
#include "ax/core/entt.hpp"
#include "ax/core/init.hpp"
#include "ax/geometry/decimate.hpp"
#include "ax/geometry/halfedge.hpp"
#include "ax/geometry/io.hpp"
#include "ax/geometry/normal.hpp"
#include "ax/gl/context.hpp"
#include "ax/gl/primitives/lines.hpp"
#include "ax/gl/primitives/mesh.hpp"
#include "ax/gl/window.hpp"
#include "ax/utils/asset.hpp"

using namespace ax;

ABSL_FLAG(std::string, obj_file, "bunny_low_res.obj", "The obj file to load");

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
    AX_CHECK(halfedge.CheckOk());
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
    auto trimesh = halfedge.ToTriangleMesh();
    modified_mesh.vertices_ = trimesh.vertices_;
    modified_mesh.indices_=trimesh.indices_;
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
  vertices = obj_result.vertices_;
  indices = obj_result.indices_;

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
  List<geo::HalfedgeEdge*> edge_to_collapse;
  AX_CHECK(he_mesh.CheckOk()) << "The mesh is not a manifold.";

  auto& mesh = add_component<gl::Mesh>(mesh_ent);

  auto trimesh = he_mesh.ToTriangleMesh();
  mesh.vertices_ = trimesh.vertices_;
  mesh.indices_ = trimesh.indices_;
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
