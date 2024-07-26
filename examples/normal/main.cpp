#include <absl/flags/flag.h>

#include "ax/core/logging.hpp"
#include "ax/core/entt.hpp"
#include "ax/core/init.hpp"
#include "ax/geometry/io.hpp"
#include "ax/geometry/normal.hpp"
#include "ax/gl/primitives/mesh.hpp"
#include "ax/gl/primitives/quiver.hpp"
#include "ax/gl/utils.hpp"
#include "ax/utils/asset.hpp"
using namespace ax;
ABSL_FLAG(std::string, obj_file, "box_naive.obj", "The obj file to load");

int main(int argc, char** argv) {
  ax::gl::init(argc, argv);
  auto file = ax::utils::get_asset("/mesh/obj/" + absl::GetFlag(FLAGS_obj_file));

  auto [vertices, indices] = geo::read_obj(file);

  {  // Setup Original
    auto original = create_entity();
    auto& mesh = add_component<gl::Mesh>(original);
    std::tie(mesh.vertices_, mesh.indices_) = std::make_pair(vertices, indices);
    mesh.normals_ = geo::normal_per_vertex(mesh.vertices_, mesh.indices_, geo::face_angle_avg);
    for (auto normal : math::each(mesh.normals_)) {
      AX_LOG(INFO) << normal.transpose();
    }

    mesh.colors_.resize(4, mesh.vertices_.cols());
    mesh.colors_.setConstant(1);
    mesh.colors_.topRows(3) = mesh.normals_;
    mesh.flush_ = mesh.use_global_model_ = true;
    mesh.is_flat_ = true;
    mesh.use_lighting_ = true;

    auto& quiver = add_component<gl::Quiver>(original);
    quiver.positions_ = mesh.vertices_;
    quiver.directions_ = mesh.normals_;
    quiver.normalize_ = true;
    quiver.colors_.resize(4, quiver.positions_.cols());
    quiver.colors_.setConstant(1);
  }

  AX_CHECK_OK(gl::enter_main_loop());

  ax::clean_up();
  return 0;
}