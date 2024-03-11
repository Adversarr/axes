#include <absl/flags/declare.h>
#include <absl/flags/flag.h>
#include <igl/readOBJ.h>
#include <imgui.h>

#include <axes/gl/utils.hpp>

#include "axes/core/echo.hpp"
#include "axes/core/entt.hpp"
#include "axes/core/init.hpp"
#include "axes/geometry/io.hpp"
#include "axes/geometry/normal.hpp"
#include "axes/geometry/sample/mesh2pc.hpp"
#include "axes/gl/context.hpp"
#include "axes/gl/primitives/lines.hpp"
#include "axes/gl/primitives/mesh.hpp"
#include "axes/gl/primitives/points.hpp"
#include "axes/utils/asset.hpp"

#include "axes/vdb/common.hpp"

#include <openvdb/tools/PointScatter.h>

using namespace ax;
ABSL_FLAG(std::string, obj_file, "bunny_low_res.obj", "The obj file to load");

Entity original, vdb_tree, reconstructed;

std::string file;
math::field3r vertices;
math::field3i indices;
math::field3r point_cloud_position, point_cloud_normal;

vdb::RealGridPtr normal_grid;


void ui_callback(gl::UiRenderEvent) {
  ImGui::Begin("Mesh to Point Cloud");
  ImGui::Text("File: %s", file.c_str());
  ImGui::Text("Vertices: %td", vertices.cols());
  ImGui::Text("Indices: %td", indices.cols());
  ImGui::Text("Point Cloud: %td", point_cloud_position.cols());
  if (ImGui::Button("Build Tree")) {
    AX_LOG(INFO) << "Building VDB tree";
    auto result = vdb::RealGrid::create();
  }
  ImGui::End();
}

int main(int argc, char** argv) {
  ax::gl::init(argc, argv);
  connect<gl::UiRenderEvent, &ui_callback>();

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

  {  // Setup Original
    original = create_entity();
    auto& mesh = add_component<gl::Mesh>(original);
    std::tie(mesh.vertices_, mesh.indices_) = std::make_pair(vertices, indices);
    mesh.normals_ = geo::normal_per_vertex(mesh.vertices_, mesh.indices_);
    mesh.colors_.resize(4, mesh.vertices_.cols());
    mesh.colors_.setConstant(1);
    mesh.colors_.topRows(3) = mesh.normals_;
    mesh.flush_ = mesh.use_global_model_ = true;
    mesh.use_lighting_ = true;

    auto& wf = add_component<gl::Lines>(original, gl::Lines::Create(mesh));  // Wireframe
    wf.colors_.setConstant(.3);

    math::field3r interp;
    interp.resize(3, 4);
    interp.block<3, 1>(0, 0) = math::vec3r(1, 0, 0);
    interp.block<3, 1>(0, 1) = math::vec3r(0, 1, 0);
    interp.block<3, 1>(0, 2) = math::vec3r(0, 0, 1);
    interp.block<3, 1>(0, 3) = math::vec3r(1.0 / 3, 1.0 / 3, 1.0 / 3);
    geo::MeshPointCloudSampler<3> sampler({vertices, indices}, interp);

    auto result = sampler.Sample<3>(vertices);
    AX_CHECK_OK(result);

    auto& point_cloud = add_component<gl::Points>(original);
    point_cloud_position = result.value();
    point_cloud_normal = sampler.Sample<3>(mesh.normals_).value();
    point_cloud.vertices_ = point_cloud_position;
    point_cloud.colors_.setOnes(4, point_cloud.vertices_.cols());
    point_cloud.colors_.topRows<3>() = point_cloud_normal;
    point_cloud.point_size_ = 3;
  }

  { // Setup VDB Normal Tree
    vdb_tree = create_entity();

    normal_grid = vdb::RealGrid::create();
    normal_grid->setName("Normal Grid");

    // openvdb::tools::DenseUniformPointScatter openvdb_scatter(*normal_grid);
    //
    openvdb::tools::DenseUniformPointScatter<vdb::Vec3rGrid, > scatter;
  }

  AX_CHECK_OK(ax::gl::enter_main_loop());
  ax::clean_up();
  return 0;
}
