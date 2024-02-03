#include "axes/gl/vao.hpp"
#define GLFW_INCLUDE_NONE
#include <GLFW/glfw3.h>
#include <absl/flags/declare.h>
#include <absl/flags/flag.h>

#include "axes/core/echo.hpp"
#include "axes/core/entt.hpp"
#include "axes/core/init.hpp"
#include "axes/gl/context.hpp"
#include "axes/gl/helpers.hpp"
#include "axes/gl/primitives/lines.hpp"
#include "axes/gl/program.hpp"
#include "axes/gl/shader.hpp"
#include "axes/gl/window.hpp"
#include "axes/utils/status.hpp"

ABSL_FLAG(bool, echo, false, "Echo events");

using namespace ax;

struct Echo {
  void WSize(const gl::WindowSizeEvent& evt) const {
    LOG(INFO) << "Window size: " << math::transpose(evt.size_);
  }

  void WPos(const gl::WindowPosEvent& evt) const {
    LOG(INFO) << "Window pos: " << math::transpose(evt.pos_);
  }

  void Key(const gl::KeyboardEvent& evt) const {
    LOG(INFO) << "Key: " << evt.key_ << std::endl << "Action: " << evt.action_;
  }
};

int main(int argc, char** argv) {
  ax::init(argc, argv);

  auto ent = create_entity();
  auto& lines = add_component<gl::Lines>(ent);
  lines.vertices_.resize(3, 2);
  lines.vertices_ << math::zeros<3>(), math::ones<3>();
  lines.vertices_ *= 0.3;
  lines.colors_ = math::ones<4>(2);
  lines.colors_.row(1).setZero();
  lines.indices_.resize(2, 1);
  lines.indices_ << 0, 1;
  lines.flush_ = true;

  auto& ctx = add_resource<gl::Context>();
  if (absl::GetFlag(FLAGS_echo)) {
    auto& echo = add_resource<Echo>();
    connect<gl::WindowSizeEvent, &Echo::WSize>(echo);
    connect<gl::WindowPosEvent, &Echo::WPos>(echo);
    connect<gl::KeyboardEvent, &Echo::Key>(echo);
  }
  // Main Loop
  auto& win = ctx.GetWindow();
  time_t start = time(nullptr);
  while (!win.ShouldClose()) {
    CHECK_OK(ctx.TickLogic());
    CHECK_OK(ctx.TickRender());

    time_t end = time(nullptr);

    double delta = difftime(end, start);
    if (delta > 5) {
      remove_component<gl::Lines>(ent);
    }
  }

  erase_resource<gl::Context>();
  clean_up();
  return 0;
}
