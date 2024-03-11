#include <absl/flags/declare.h>
#include <absl/flags/flag.h>
#include <igl/readOBJ.h>
#include <imgui.h>
#include <openvdb/points/AttributeArray.h>
#include <openvdb/points/PointConversion.h>
#include <openvdb/points/PointCount.h>
#include <openvdb/points/PointRasterizeTrilinear.h>
#include <openvdb/tools/GridOperators.h>
#include <openvdb/tools/VolumeToMesh.h>

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

using namespace ax;
ABSL_FLAG(std::string, obj_file, "bunny_low_res.obj", "The obj file to load");

Entity original, vdb_tree, reconstructed;

std::string file;
math::field3r vertices;
math::field3i indices;
math::field3r point_cloud_position, point_cloud_normal;

real voxel_size;
vdb::Vec3rGridPtr normal_grid;
openvdb::points::PointDataGrid::Ptr point_data_grid;
openvdb::tools::PointIndexGrid::Ptr point_index_grid;
openvdb::math::Transform::Ptr transform;

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

  {  // Setup VDB Normal Tree
    std::vector<openvdb::Vec3R> positions, normals;
    positions.reserve(point_cloud_position.cols());
    normals.reserve(point_cloud_normal.cols());
    for (auto p : math::each(point_cloud_position)) {
      positions.push_back(openvdb::Vec3R(p.x(), p.y(), p.z()));
    }
    for (auto n : math::each(point_cloud_normal)) {
      normals.push_back(openvdb::Vec3R(n.x(), n.y(), n.z()));
    }

    openvdb::points::PointAttributeVector<openvdb::Vec3R> position_wrapper(positions);
    openvdb::points::PointAttributeVector<openvdb::Vec3R> normals_wrapper(normals);

    int points_per_voxel = 8;
    voxel_size = openvdb::points::computeVoxelSize(position_wrapper, points_per_voxel);
    transform = openvdb::math::Transform::createLinearTransform(voxel_size);

    point_index_grid = openvdb::tools::createPointIndexGrid<openvdb::tools::PointIndexGrid>(
        position_wrapper, *transform);
    point_data_grid = openvdb::points::createPointDataGrid<openvdb::points::NullCodec,
                                                           openvdb::points::PointDataGrid>(
        *point_index_grid, position_wrapper, *transform);
    point_data_grid->setName("Points");
    using Codec = openvdb::points::NullCodec;
    openvdb::points::TypedAttributeArray<vdb::Vec3r, Codec>::registerType();
    openvdb::NamePair normal_attrribute
        = openvdb::points::TypedAttributeArray<vdb::Vec3r, Codec>::attributeType();
    openvdb::points::appendAttribute(point_data_grid->tree(), "normal", normal_attrribute);
    openvdb::points::populateAttribute(point_data_grid->tree(), point_index_grid->tree(), "normal",
                                       normals_wrapper);
    auto normal_tree = openvdb::DynamicPtrCast<vdb::Vec3rTree>(
        openvdb::points::rasterizeTrilinear<true, vdb::Vec3r>(point_data_grid->tree(), "normal"));
    normal_grid = vdb::Vec3rGrid::create(normal_tree)->deepCopy();
    normal_grid->setTransform(transform);
  }
  
  {
    vdb_tree = create_entity();
    idx active_cnt = normal_grid->activeVoxelCount();
    auto& points = add_component<gl::Points>(vdb_tree);
    points.colors_.resize(4, active_cnt);
    points.colors_.setConstant(1);
    points.vertices_.resize(3, active_cnt);
    points.point_size_ = 3;
    idx cnt = 0;
    for (auto iter = normal_grid->beginValueOn(); iter.test(); ++iter) {
      auto value = *iter;
      AX_LOG(INFO) << "Coord: " << iter.getCoord() << "Value: " << value;
      auto position = transform->indexToWorld(iter.getCoord());
      points.vertices_.col(cnt++) = math::vec3r(position.x(), position.y(), position.z());
    }
  }

  {
    auto divergence = openvdb::tools::divergence(*normal_grid);
    openvdb::tools::VolumeToMesh mesher(0, 0, true);
    mesher(*divergence);

    reconstructed = create_entity();
    auto& mesh = add_component<gl::Points>(reconstructed);
    idx n_points = mesher.pointListSize();
    mesh.vertices_.resize(3, n_points);
    idx n_triangles = mesher.polygonPoolListSize();
    for (idx i = 0; i < n_points; ++i) {
      auto p = mesher.pointList()[i];
      mesh.vertices_.col(i) = math::vec3r(p.x(), p.y(), p.z());
    }
    mesh.colors_.resize(4, n_points);
    mesh.colors_.setConstant(1);
  }
  AX_CHECK_OK(ax::gl::enter_main_loop());
  ax::clean_up();
  return 0;
}
