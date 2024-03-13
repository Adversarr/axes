#include <absl/flags/declare.h>
#include <absl/flags/flag.h>
#include <igl/readOBJ.h>
#include <imgui.h>
#include <openvdb/tools/GridOperators.h>
#include <openvdb/tools/VolumeToMesh.h>
#include <axes/gl/utils.hpp>

#include "axes/components/name.hpp"
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
#include "axes/gl/primitives/quiver.hpp"
#include "axes/utils/asset.hpp"
#include "axes/vdb/pointcloud.hpp"
#include "axes/vdb/volumetomesh.hpp"
#include "solve_poisson.hpp"

#undef ERROR

using namespace ax;
ABSL_FLAG(std::string, obj_file, "box_naive.obj", "The obj file to load");
ABSL_FLAG(real, voxel_size, -1, "The voxel size for the point cloud");
ABSL_FLAG(idx, sample, 10, "Number of points sampled each face");

Entity original, vdb_tree, reconstructed;

std::string file;
math::field3r vertices;
math::field3i indices;
math::field3r point_cloud_position, point_cloud_normal;

float ratio = 0;

idx N_sample = 10;

real voxel_size;
vdb::Vec3rGridPtr normal_grid;
openvdb::points::PointDataGrid::Ptr point_data_grid;
openvdb::tools::PointIndexGrid::Ptr point_index_grid;
openvdb::math::Transform::Ptr transform;

vdb::RealGridPtr distance;

void ui_callback(gl::UiRenderEvent) {
  ImGui::Begin("Mesh to Point Cloud");
  ImGui::Text("File: %s", file.c_str());
  ImGui::Text("Vertices: %td", vertices.cols());
  ImGui::Text("Indices: %td", indices.cols());
  ImGui::Text("Point Cloud: %td", point_cloud_position.cols());
  real min = -50.0, max = 50.0;
  if (ImGui::SliderFloat("Ratio", &ratio, min, max)) {
    auto& mesh = get_component<gl::Mesh>(reconstructed);
    vdb::VolumeToMesh mesher(ratio);
    std::tie(mesh.vertices_, mesh.indices_) = mesher(distance);
    mesh.vertices_.row(0) *= transform->voxelSize().x();
    mesh.vertices_.row(1) *= transform->voxelSize().y();
    mesh.vertices_.row(2) *= transform->voxelSize().z();

    mesh.normals_ = geo::normal_per_vertex(mesh.vertices_, mesh.indices_);
    mesh.colors_.resize(4, mesh.vertices_.cols());
    mesh.colors_.setConstant(0.5);
    mesh.flush_ = true;
  }
  ImGui::End();
}

int main(int argc, char** argv) {
  ax::gl::init(argc, argv);
  connect<gl::UiRenderEvent, &ui_callback>();

  openvdb::initialize();

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
    original = cmpt::create_named_entity("Input Mesh");
    auto& mesh = add_component<gl::Mesh>(original);
    std::tie(mesh.vertices_, mesh.indices_) = std::make_pair(vertices, indices);
    mesh.normals_ = geo::normal_per_vertex(mesh.vertices_, mesh.indices_, geo::face_angle_avg);
    mesh.colors_.resize(4, mesh.vertices_.cols());
    mesh.colors_.setConstant(1);
    mesh.colors_.topRows(3) = mesh.normals_;
    mesh.flush_ = mesh.use_global_model_ = true;
    mesh.use_lighting_ = true;

    auto& wf = add_component<gl::Lines>(original, gl::Lines::Create(mesh));  // Wireframe
    wf.colors_.setConstant(.3);
    mesh.flush_ = false;

    math::field3r interp;
    N_sample = absl::GetFlag(FLAGS_sample);
    interp.resize(3, N_sample);
    for (auto p: math::each(interp)) {
      real x = rand() / ((real) RAND_MAX);
      real y = rand() / ((real) RAND_MAX) * (1 - x);
      real z = 1 - x - y;
      p = math::vec3r(x, y, z);
    }

    geo::MeshPointCloudSampler<3> sampler({vertices, indices}, interp);

    auto result = sampler.Sample<3>(vertices);
    AX_CHECK_OK(result);

    point_cloud_position = result.value();
    point_cloud_normal = sampler.Sample<3>(mesh.normals_).value();
    for (auto n : math::each(point_cloud_normal)) {
      n.normalize();
    }
  }
  real voxel_size = absl::GetFlag(FLAGS_voxel_size);
  vdb::PointGrid pg(point_cloud_position, voxel_size, 8);

  point_data_grid = pg.DataGrid();
  point_index_grid = pg.IndexGrid();
  transform = pg.Transform();

  ax::math::field1r ones = math::ones<1>(point_cloud_position.cols());
  normal_grid = pg.TransferCellCenter("normal", point_cloud_normal);
  auto ones_grid = pg.TransferCellCenter("ones", ones);

  { // Visualize the normal grid
    idx active_cnt = normal_grid->activeVoxelCount();
    auto &quiver = add_component<gl::Quiver>(original);
    quiver.positions_.resize(3, active_cnt);
    quiver.directions_.resize(3, active_cnt);
    idx cnt = 0;
    for (auto iter = normal_grid->beginValueOn(); iter; ++iter) {
      auto val = *iter;
      auto position = transform->indexToWorld(iter.getCoord());
      quiver.positions_.col(cnt) = math::vec3r(position.x(), position.y(), position.z());
      quiver.directions_.col(cnt) = math::vec3r(val.x(), val.y(), val.z());
      cnt++;
    }
    quiver.colors_.resize(4, active_cnt);
    quiver.colors_.setConstant(1);
    quiver.scale_ = transform->voxelSize().x();
  }

  for (auto iter = normal_grid->beginValueOn(); iter; ++iter) {
    auto val = *iter;
    val.normalize();
    iter.setValue(val);
  }


  {
    vdb_tree = cmpt::create_named_entity("Vdb Tree");
    idx active_cnt = normal_grid->activeVoxelCount();
    auto& points = add_component<gl::Points>(vdb_tree);
    points.colors_.resize(4, active_cnt);
    points.colors_.setConstant(1);
    points.vertices_.resize(3, active_cnt);
    points.point_size_ = 3;
    idx cnt = 0;
    for (auto iter = normal_grid->beginValueOn(); iter.test(); ++iter) {
      auto position = transform->indexToWorld(iter.getCoord());
      points.vertices_.col(cnt++) = math::vec3r(position.x(), position.y(), position.z());
    }
  }

  auto divergence = openvdb::tools::divergence(*normal_grid);
  divergence->tree().voxelizeActiveTiles();
  openvdb::CoordBBox bbox = divergence->evalActiveVoxelBoundingBox();

  // auto divergence_filled = vdb::RealGrid::create();
  // divergence_filled->fill(bbox, 0);
  // for (auto iter = divergence->beginValueOn(); iter; ++iter) {
  //   divergence_filled->tree().setValue(iter.getCoord(), *iter);
  // }
  // divergence_filled->setTransform(transform);

  AX_LOG(INFO) << "Lower Bounds: " << bbox.min();
  AX_LOG(INFO) << "Upper Bounds: " << bbox.max();


  distance = solve_poisson(divergence);
  vdb::VolumeToMesh mesher(ratio);
  auto mesh = mesher(distance);

  reconstructed = cmpt::create_named_entity("Reconstructed Mesh");
  auto& mesh_reconstructed = add_component<gl::Mesh>(reconstructed);
  std::tie(mesh_reconstructed.vertices_, mesh_reconstructed.indices_) = mesh;

  mesh_reconstructed.vertices_.row(0) *= transform->voxelSize().x();
  mesh_reconstructed.vertices_.row(1) *= transform->voxelSize().y();
  mesh_reconstructed.vertices_.row(2) *= transform->voxelSize().z();

  mesh_reconstructed.normals_
      = geo::normal_per_vertex(mesh_reconstructed.vertices_, mesh_reconstructed.indices_);
  mesh_reconstructed.colors_.resize(4, mesh_reconstructed.vertices_.cols());
  mesh_reconstructed.colors_.setConstant(0.5);
  mesh_reconstructed.use_lighting_ = true;
  mesh_reconstructed.flush_ = true;

  AX_CHECK_OK(ax::gl::enter_main_loop());
  point_data_grid.reset();
  point_index_grid.reset();
  transform.reset();
  distance.reset();
  normal_grid.reset();
  ax::clean_up();
  return 0;
}
