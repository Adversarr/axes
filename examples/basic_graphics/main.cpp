#include <absl/flags/declare.h>
#include <absl/flags/flag.h>
#include <imgui.h>

#include "axes/core/echo.hpp"
#include "axes/core/entt.hpp"
#include "axes/core/init.hpp"
#include "axes/geometry/normal.hpp"
#include "axes/geometry/primitives.hpp"
#include "axes/geometry/transforms.hpp"
#include "axes/gl/context.hpp"
#include "axes/gl/extprim/axes.hpp"
#include "axes/gl/primitives/lines.hpp"
#include "axes/gl/primitives/mesh.hpp"
#include "axes/gl/window.hpp"
#include "axes/utils/time.hpp"

ABSL_FLAG(bool, echo, false, "Echo events");
ABSL_FLAG(bool, rotate, false, "Rotate the cube");

using namespace ax;

struct Echo {
  void WSize(const gl::WindowSizeEvent& evt) const { LOG(INFO) << "Window size: " << math::transpose(evt.size_); }

  void WPos(const gl::WindowPosEvent& evt) const { LOG(INFO) << "Window pos: " << math::transpose(evt.pos_); }

  void Key(const gl::KeyboardEvent& evt) const {
    LOG(INFO) << "Key: " << evt.key_ << std::endl << "Action: " << evt.action_;
  }
};

void render_ui_dummy() { ImGui::ShowDemoWindow(); }

gl::Lines create_dummy_line() {
  gl::Lines lines;
  lines.vertices_.resize(3, 3);
  lines.vertices_.col(0) = math::vec3r::Zero();
  lines.vertices_.col(1) = math::vec3r{0.5, 0.5, 0.};
  lines.vertices_.col(2) = math::vec3r{0.5, -0.5, 0.};
  lines.vertices_.colwise() += math::vec3r::UnitY();
  lines.colors_.resize(4, 3);
  lines.colors_.setRandom();
  lines.colors_ = lines.colors_ * 0.5;
  lines.colors_.colwise() += math::ones<4>();
  lines.indices_.resize(2, 2);
  lines.indices_.col(0) = math::vec2i{0, 1};
  lines.indices_.col(1) = math::vec2i{1, 2};
  lines.flush_ = true;
  return lines;
}

gl::Mesh create_dummy_cube() {
  gl::Mesh mesh;
  std::tie(mesh.vertices_, mesh.indices_) = geo::cube(0.5);
  mesh.colors_.setRandom(4, 8);

  mesh.vertices_ *= 0.7;

  mesh.normals_ = geo::normal_per_vertex(mesh.vertices_, mesh.indices_);
  mesh.colors_ = (mesh.colors_.array() * 0.5 + 0.5).matrix();
  mesh.flush_ = true;

  mesh.instance_offset_.resize(3, 3);
  mesh.instance_offset_.col(0) = math::vec3r{0.5, 0.5, 0};
  mesh.instance_offset_.col(1) = math::vec3r{-0.5, 0.5, 0};
  mesh.instance_offset_.col(2) = math::vec3r{0.5, -0.5, 0};
  mesh.instance_color_.setZero(4, 3);

  return mesh;
}

gl::Mesh create_dummy_sphere() {
  gl::Mesh mesh;
  std::tie(mesh.vertices_, mesh.indices_) = geo::sphere(0.5, 10, 10);
  math::each(mesh.vertices_) += math::vec3r{1.5, 0, 0};
  mesh.colors_.setConstant(4, mesh.vertices_.cols(), 0);
  mesh.colors_.row(0) = mesh.vertices_.row(0) * 0.5;
  mesh.normals_ = geo::normal_per_vertex(mesh.vertices_, mesh.indices_);
  mesh.use_lighting_ = true;
  mesh.is_flat_ = true;
  mesh.flush_ = true;
  return mesh;
}

int main(int argc, char** argv) {
  ax::init(argc, argv);
  auto& ctx = add_resource<gl::Context>();

  // SECT: Connect the echo events if the flag is set
  if (absl::GetFlag(FLAGS_echo)) {
    auto& echo = add_resource<Echo>();
    connect<gl::WindowSizeEvent, &Echo::WSize>(echo);
    connect<gl::WindowPosEvent, &Echo::WPos>(echo);
    connect<gl::KeyboardEvent, &Echo::Key>(echo);
  }

  // SECT: Connect the ui rendering test function to UiRenderEvent
  connect<gl::UiRenderEvent, &render_ui_dummy>();

  // SECT: This is a dummy entity to test the rendering pipeline
  // auto& lines = add_component<gl::Lines>(create_entity(), create_dummy_line());  // Line
  auto mesh_ent = create_entity();
  auto& mesh = add_component<gl::Mesh>(mesh_ent, create_dummy_cube());                 // Mesh
  auto& mesh_wireframe = add_component<gl::Lines>(mesh_ent, gl::Lines::Create(mesh));  // Wireframe
  mesh_wireframe.colors_.setOnes();
  mesh_wireframe.instance_color_.setOnes();

  auto& sphere = add_component<gl::Mesh>(create_entity(), create_dummy_sphere());  // Sphere
  auto& sphere_wireframe = add_component<gl::Lines>(create_entity(), gl::Lines::Create(sphere));
  sphere_wireframe.colors_.setZero();

  // SECT: Main Loop
  auto& win = ctx.GetWindow();
  i64 start = utils::GetCurrentTimeNanos();

  ctx.GetCamera().SetProjectionMode(true);
  mesh.use_lighting_ = true;
  mesh.is_flat_ = false;

  bool rotate = absl::GetFlag(FLAGS_rotate);

  while (!win.ShouldClose()) {
    CHECK_OK(ctx.TickLogic());
    CHECK_OK(ctx.TickRender());

    i64 current = utils::GetCurrentTimeNanos();
    real dt = (current - start) / 1.0e9;
    if (rotate) {
      math::mat4r model = geo::rotate_y(dt * 0.3);
      ctx.SetGlobalModelMatrix(model.cast<f32>());
    }
  }

  // NOTE: Clean Up
  erase_resource<gl::Context>();
  clean_up();
  return 0;
}
