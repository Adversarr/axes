#include <absl/flags/declare.h>
#include <absl/flags/flag.h>
#include <imgui.h>

#include "ax/math/utils/formatting.hpp"
#include "ax/core/entt.hpp"
#include "ax/core/init.hpp"
#include "ax/core/logging.hpp"
#include "ax/geometry/normal.hpp"
#include "ax/geometry/primitives.hpp"
#include "ax/geometry/transforms.hpp"
#include "ax/gl/colormap.hpp"
#include "ax/gl/context.hpp"
#include "ax/gl/extprim/axes.hpp"
#include "ax/gl/primitives/lines.hpp"
#include "ax/gl/primitives/mesh.hpp"
#include "ax/gl/primitives/quiver.hpp"
#include "ax/gl/window.hpp"
#include "ax/utils/time.hpp"

using namespace ax;

struct Echo {
  void WSize(const gl::WindowSizeEvent& evt) const {
    // AX_LOG(INFO) << "Window size: " << math::transpose(evt.size_);
    AX_INFO("Window size: {}", evt.size_);
  }

  void WPos(const gl::WindowPosEvent& evt) const { 
    // AX_LOG(INFO) << "Window pos: " << math::transpose(evt.pos_);
    AX_INFO("Window pos: {}", evt.pos_);
  }

  void Key(const gl::KeyboardEvent& evt) const {
    AX_INFO("Key: {} Action: {}", evt.key_, evt.action_);
  }
};

void render_ui_dummy() { ImGui::ShowDemoWindow(); }

gl::Lines create_dummy_line() {
  gl::Lines lines;
  lines.vertices_.resize(3, 3);
  lines.vertices_.col(0) = math::RealVector3::Zero();
  lines.vertices_.col(1) = math::RealVector3{0.5, 0.5, 0.};
  lines.vertices_.col(2) = math::RealVector3{0.5, -0.5, 0.};
  lines.vertices_.colwise() += math::RealVector3::UnitY();
  lines.colors_.resize(4, 3);
  lines.colors_.setRandom();
  lines.colors_ = lines.colors_ * 0.5;
  lines.colors_.colwise() += math::ones<4>();
  lines.indices_.resize(2, 2);
  lines.indices_.col(0) = math::IndexVec2{0, 1};
  lines.indices_.col(1) = math::IndexVec2{1, 2};
  lines;
  return lines;
}

gl::Mesh create_dummy_cube() {
  gl::Mesh mesh;
  auto cube = geo::cube(0.5);
  mesh.vertices_ = cube.vertices_;
  mesh.indices_ = cube.indices_;
  mesh.colors_.setRandom(4, 8);

  mesh.vertices_ *= 0.7;

  mesh.normals_ = geo::normal_per_vertex(mesh.vertices_, mesh.indices_);
  mesh.colors_ = (mesh.colors_.array() * 0.5 + 0.5).matrix();
  mesh;

  mesh.instance_offset_.resize(3, 3);
  mesh.instance_offset_.col(0) = math::RealVector3{0.5, 0.5, 0};
  mesh.instance_offset_.col(1) = math::RealVector3{-0.5, 0.5, 0};
  mesh.instance_offset_.col(2) = math::RealVector3{0.5, -0.5, 0};
  mesh.instance_color_.setZero(4, 3);

  return mesh;
}

gl::Mesh create_dummy_sphere() {
  gl::Mesh mesh;
  auto sp = geo::sphere(0.5, 10, 10);
  mesh.vertices_ = sp.vertices_;
  mesh.indices_ = sp.indices_;
  math::each(mesh.vertices_) += math::RealVector3{1.5, 0, 0};
  mesh.colors_.resize(4, mesh.vertices_.cols());
  for (Index i = 0; i < mesh.vertices_.cols(); ++i) {
    mesh.colors_.topRows<3>().col(i) = math::RealVector3(gl::colormap_coolwarm[i % 256]);
  }
  mesh.normals_ = geo::normal_per_vertex(mesh.vertices_, mesh.indices_);
  mesh.use_lighting_ = true;
  mesh.is_flat_ = true;
  mesh;
  return mesh;
}

int main(int argc, char** argv) {
  ax::init(argc, argv);
  auto& ctx = add_resource<gl::Context>();

  // SECT: Connect the echo events if the flag is set
  auto& echo = add_resource<Echo>();
  connect<gl::WindowSizeEvent, &Echo::WSize>(echo);
  connect<gl::WindowPosEvent, &Echo::WPos>(echo);
  connect<gl::KeyboardEvent, &Echo::Key>(echo);

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
  i64 start = utils::get_current_time_nanos();

  ctx.GetCamera().SetProjectionMode(true);
  mesh.use_lighting_ = true;
  mesh.is_flat_ = false;

  {
    auto ent = create_entity();
    add_component<gl::Quiver>(ent);  // Quiver
    patch_component<gl::Quiver>(ent,[&](gl::Quiver & quiver) {
      quiver;
      quiver.positions_.resize(3, 4);
      quiver.positions_.setRandom();
      quiver.directions_ = quiver.positions_;
      quiver.colors_.resize(4, 4);
      quiver.colors_.setConstant(1);
      quiver.colors_.row(1).setConstant(0.3);
    });
  }


  bool rotate = true;

  while (!win.ShouldClose()) {
    ctx.TickLogic();
    ctx.TickRender();

    i64 current = utils::get_current_time_nanos();
    Real dt = (current - start) / 1.0e9;
    if (rotate) {
      math::RealMatrix4 model = geo::rotate_y(dt * 0.3);
      ctx.SetGlobalModelMatrix(model.cast<f32>());
    }
  }

  // NOTE: Clean Up
  erase_resource<gl::Context>();
  clean_up();
  return 0;
}
