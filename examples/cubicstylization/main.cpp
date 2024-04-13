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
Entity mesh_ent, original_ent;
std::string file;
ABSL_FLAG(std::string, obj_file, "sphere_naive.obj", "The obj file to load");


UPtr<Solver> solver{nullptr};

void reload_file() {
  auto obj_result = geo::read_obj( utils::get_asset("/mesh/obj/" + file));
  if (! obj_result.ok()) {
    AX_LOG(ERROR) << "Failed to load obj: " << std::quoted(file);
    return;
  }

  auto& mesh = get_component<geo::SurfaceMesh>(mesh_ent) = obj_result.value();

  auto& original_mesh_render = get_component<gl::Mesh>(original_ent);
  original_mesh_render.vertices_ = mesh.vertices_;
  original_mesh_render.indices_ = mesh.indices_;
  original_mesh_render.normals_ = geo::normal_per_vertex(mesh.vertices_, mesh.indices_);
  original_mesh_render.colors_.setConstant(4, mesh.vertices_.cols(), 0.6);
  original_mesh_render.flush_= true;
  original_mesh_render.use_lighting_ = true;



  auto & stylized_mesh_render = add_or_replace_component<gl::Mesh>(mesh_ent, original_mesh_render);
  add_or_replace_component<gl::Lines>(mesh_ent, gl::Lines::Create(stylized_mesh_render));
  solver = std::make_unique<Solver>(obj_result.value());

  original_mesh_render.vertices_.row(2).array() += 1;
  add_or_replace_component<gl::Lines>(original_ent, gl::Lines::Create(original_mesh_render));
}

void ui_callback(gl::UiRenderEvent) {
  ImGui::Begin("Cubic Stylization");
  static int steps = 1, dp;
  ImGui::InputInt("Max Iteration", &steps);
  ImGui::InputDouble("rho", &solver->rho_);
  ImGui::InputDouble("lambda", &solver->lambda_);
  ImGui::InputDouble("tau", &solver->tau_);
  ImGui::InputDouble("mu", &solver->mu_);
  if (ImGui::SliderInt("Display Iteration", &dp, 0, (int) solver->cached_sequence.size() - 1)) {
        auto& stylized_mesh_render = get_component<gl::Mesh>(mesh_ent);
        stylized_mesh_render.vertices_ = solver->cached_sequence[dp];
        stylized_mesh_render.flush_ = true;

        add_or_replace_component<gl::Lines>(mesh_ent, gl::Lines::Create(stylized_mesh_render))
        .flush_ = true;
  }

  if (ImGui::Button("Step Once")) {
    solver->Step(steps);
    auto& stylized_mesh_render = get_component<gl::Mesh>(mesh_ent);
    stylized_mesh_render.vertices_ = solver->GetResult().vertices_;
    stylized_mesh_render.flush_ = true;

    add_or_replace_component<gl::Lines>(mesh_ent, gl::Lines::Create(stylized_mesh_render))
    .flush_ = true;
  }

  static char buffer[LINE_MAX];
  if (ImGui::InputText("File", buffer, LINE_MAX, ImGuiInputTextFlags_EnterReturnsTrue)) {
    file = buffer;
    AX_LOG(INFO) << "Loading file: " << std::quoted(file);
  }
  if (ImGui::Button("Reload")) {
    reload_file();
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
  original_ent = cmpt::create_named_entity("Original");
  auto & original_mesh_render = add_component<gl::Mesh>(original_ent);

  original_mesh_render.vertices_ = mesh.vertices_;
  original_mesh_render.indices_ = mesh.indices_;
  original_mesh_render.normals_ = geo::normal_per_vertex(mesh.vertices_, mesh.indices_);
  original_mesh_render.colors_.setConstant(4, mesh.vertices_.cols(), 0.6);
  original_mesh_render.use_lighting_ = true;


  // Render stylized one:
  auto & stylized_mesh_render = add_component<gl::Mesh>(mesh_ent, original_mesh_render);
  add_component<gl::Lines>(mesh_ent, gl::Lines::Create(stylized_mesh_render));
  solver = std::make_unique<Solver>(obj_result.value());
  
  original_mesh_render.vertices_.row(2).array() += 1;
  add_component<gl::Lines>(original_ent, gl::Lines::Create(original_mesh_render));
  connect<gl::UiRenderEvent, &ui_callback>();

  AX_CHECK_OK(gl::enter_main_loop());
  clean_up();
  return 0;
}
