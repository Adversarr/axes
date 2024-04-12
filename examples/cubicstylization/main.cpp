#include <imgui.h>
#include <iomanip>

#include "ax/core/entt.hpp"
#include "ax/core/init.hpp"
#include "ax/geometry/io.hpp"
#include "ax/gl/context.hpp"
#include "ax/gl/primitives/mesh.hpp"
#include "ax/gl/primitives/lines.hpp"
#include "ax/gl/utils.hpp"
#include "ax/utils/asset.hpp"
#include "ax/utils/status.hpp"
#include "ax/components/name.hpp"
#include "ax/geometry/normal.hpp"
#include "stylization.hpp"

using namespace ax;
Entity mesh_ent;
std::string file;
ABSL_FLAG(std::string, obj_file, "sphere_naive.obj", "The obj file to load");

void ui_callback(gl::UiRenderEvent) {
  ImGui::Begin("Cubic Stylization");
  static int steps = 1;
  ImGui::InputInt("Max Iteration", &steps);
  if (ImGui::Button("Step Once")) {
    auto const& m = get_component<SurfaceMesh>(mesh_ent);
    Solver solver{m};
    solver.Step(steps);

    auto& stylized_mesh_render = get_component<gl::Mesh>(mesh_ent);
    stylized_mesh_render.vertices_ = solver.GetResult().vertices_;
    stylized_mesh_render.flush_ = true;

    add_or_replace_component<gl::Lines>(mesh_ent, gl::Lines::Create(stylized_mesh_render))
    .flush_ = true;
  }
  ImGui::End();
}



int main(int argc, char** argv) {
  ax::gl::init(argc, argv);
  mesh_ent = create_entity();
  file = utils::get_asset("/mesh/obj/" + absl::GetFlag(FLAGS_obj_file));
  auto obj_result = geo::read_obj(file);
  if (! obj_result.ok()) {
    AX_LOG(ERROR) << "Failed to load obj: " << std::quoted(file);
    return -1;
  }

  auto& mesh = add_component<geo::SurfaceMesh>(mesh_ent, obj_result.value());

  // Render original one:
  auto & original_mesh_render = add_component<gl::Mesh>(cmpt::create_named_entity("Original"));

  original_mesh_render.vertices_ = mesh.vertices_;
  original_mesh_render.indices_ = mesh.indices_;
  original_mesh_render.normals_ = geo::normal_per_vertex(mesh.vertices_, mesh.indices_);
  original_mesh_render.colors_.setConstant(4, mesh.vertices_.cols(), 0.6);
  original_mesh_render.flush_= true;
  original_mesh_render.use_lighting_ = true;
  original_mesh_render.is_flat_ = true;

  // Render stylized one:
  auto & stylized_mesh_render = add_component<gl::Mesh>(mesh_ent, original_mesh_render);
  
  
  original_mesh_render.vertices_.row(2).array() += 1.5;
  connect<gl::UiRenderEvent, &ui_callback>();

  AX_CHECK_OK(gl::enter_main_loop());
  clean_up();
  return 0;
}
